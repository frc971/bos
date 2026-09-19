#include "src/tools/path_camera_sim/gamepiece_config.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>

#include <frc/geometry/Rotation3d.h>
#include <units/angle.h>
#include <units/length.h>
#include <Eigen/Geometry>

#include "nlohmann/json.hpp"
#include "src/tools/path_camera_sim/glb_utils.h"

namespace path_camera_sim {
namespace {

constexpr double kInchesToMeters = 0.0254;
constexpr double kFuelRadiusMeters = 0.075;
constexpr double kMaximumSpreadMeters = 2.0;
constexpr double kMaximumMissingFraction = 0.5;

auto ReadJson(const std::filesystem::path& path) -> nlohmann::json {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open JSON file: " + path.string());
  }
  nlohmann::json result;
  input >> result;
  return result;
}

auto RotationMatrix(const std::string& axis, double radians)
    -> Eigen::Matrix4d {
  Eigen::Vector3d vector;
  if (axis == "x") {
    vector = Eigen::Vector3d::UnitX();
  } else if (axis == "y") {
    vector = Eigen::Vector3d::UnitY();
  } else if (axis == "z") {
    vector = Eigen::Vector3d::UnitZ();
  } else {
    throw std::runtime_error("Unsupported model rotation axis: " + axis);
  }
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(radians, vector).toRotationMatrix();
  return result;
}

auto ConfigModelTransform(const nlohmann::json& object) -> Eigen::Matrix4d {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  for (const auto& rotation :
       object.value("rotations", nlohmann::json::array())) {
    result =
        RotationMatrix(rotation.at("axis").get<std::string>(),
                       rotation.at("degrees").get<double>() * M_PI / 180.0) *
        result;
  }
  if (object.contains("position")) {
    const auto& position = object.at("position");
    result(0, 3) = position.at(0).get<double>();
    result(1, 3) = position.at(1).get<double>();
    result(2, 3) = position.at(2).get<double>();
  }
  return result;
}

auto FieldModelToWpilib(double field_length_meters, double field_width_meters)
    -> Eigen::Matrix4d {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result(0, 0) = -1.0;
  result(1, 1) = -1.0;
  result(0, 3) = field_length_meters / 2.0;
  result(1, 3) = field_width_meters / 2.0;
  return result;
}

auto NodeTransform(const nlohmann::json& node) -> Eigen::Matrix4d {
  if (node.contains("matrix")) {
    const auto& values = node.at("matrix");
    if (!values.is_array() || values.size() != 16) {
      throw std::runtime_error("GLB node matrix must have 16 elements");
    }
    Eigen::Matrix4d result;
    // glTF stores matrices in column-major order.
    for (size_t column = 0; column < 4; ++column) {
      for (size_t row = 0; row < 4; ++row) {
        result(row, column) = values.at(column * 4 + row).get<double>();
      }
    }
    return result;
  }

  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  if (node.contains("translation")) {
    const auto& value = node.at("translation");
    result.translate(Eigen::Vector3d(value.at(0).get<double>(),
                                     value.at(1).get<double>(),
                                     value.at(2).get<double>()));
  }
  if (node.contains("rotation")) {
    const auto& value = node.at("rotation");
    const Eigen::Quaterniond rotation(
        value.at(3).get<double>(), value.at(0).get<double>(),
        value.at(1).get<double>(), value.at(2).get<double>());
    result.rotate(rotation.normalized());
  }
  if (node.contains("scale")) {
    const auto& value = node.at("scale");
    result.scale(Eigen::Vector3d(value.at(0).get<double>(),
                                 value.at(1).get<double>(),
                                 value.at(2).get<double>()));
  }
  return result.matrix();
}

void FindNodeTransforms(const nlohmann::json& nodes, size_t node_index,
                        const Eigen::Matrix4d& parent_transform,
                        const std::string& target_name,
                        std::vector<Eigen::Matrix4d>& result) {
  const auto& node = nodes.at(node_index);
  const Eigen::Matrix4d world_transform =
      parent_transform * NodeTransform(node);
  if (node.value("name", std::string{}) == target_name &&
      node.contains("mesh")) {
    result.push_back(world_transform);
  }
  for (const auto& child : node.value("children", nlohmann::json::array())) {
    FindNodeTransforms(nodes, child.get<size_t>(), world_transform, target_name,
                       result);
  }
}

auto RootNodeName(const std::filesystem::path& glb_path) -> std::string {
  const auto document = ReadGlbJson(glb_path);
  const size_t scene_index = document.value("scene", 0U);
  const auto& roots = document.at("scenes").at(scene_index).at("nodes");
  if (roots.size() != 1) {
    throw std::runtime_error(
        "Gamepiece GLB must contain exactly one root node: " +
        glb_path.string());
  }
  return document.at("nodes")
      .at(roots.at(0).get<size_t>())
      .at("name")
      .get<std::string>();
}

void ValidateEntropy(double entropy) {
  if (!std::isfinite(entropy) || entropy < 0.0 || entropy > 1.0) {
    throw std::runtime_error("Entropy must be a finite number from 0 to 1");
  }
}

auto PoseJson(const GamepiecePose& gamepiece) -> nlohmann::json {
  return {{"type", gamepiece.type},
          {"translation_m",
           {gamepiece.pose.X().value(), gamepiece.pose.Y().value(),
            gamepiece.pose.Z().value()}},
          {"rotation_rpy_rad",
           {gamepiece.pose.Rotation().X().value(),
            gamepiece.pose.Rotation().Y().value(),
            gamepiece.pose.Rotation().Z().value()}}};
}

}  // namespace

auto ReadGamepieceConfig(const std::filesystem::path& path)
    -> std::vector<GamepiecePose> {
  const auto json = ReadJson(path);
  const auto& items = json.at("gamepieces");
  if (!items.is_array()) {
    throw std::runtime_error("'gamepieces' must be an array in " +
                             path.string());
  }

  std::vector<GamepiecePose> result;
  result.reserve(items.size());
  for (const auto& item : items) {
    const auto& translation = item.at("translation_m");
    const auto rotation =
        item.value("rotation_rpy_rad", nlohmann::json::array({0.0, 0.0, 0.0}));
    if (!translation.is_array() || translation.size() != 3 ||
        !rotation.is_array() || rotation.size() != 3) {
      throw std::runtime_error(
          "Gamepiece translations and rotations must have three elements");
    }
    const double x = translation.at(0).get<double>();
    const double y = translation.at(1).get<double>();
    const double z = translation.at(2).get<double>();
    const double roll = rotation.at(0).get<double>();
    const double pitch = rotation.at(1).get<double>();
    const double yaw = rotation.at(2).get<double>();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
      throw std::runtime_error("Gamepiece poses must contain finite numbers");
    }
    result.push_back(
        {.type = item.at("type").get<std::string>(),
         .pose = frc::Pose3d(
             units::meter_t{x}, units::meter_t{y}, units::meter_t{z},
             frc::Rotation3d(units::radian_t{roll}, units::radian_t{pitch},
                             units::radian_t{yaw}))});
  }
  return result;
}

auto LoadGamepieces(const std::filesystem::path& path)
    -> std::vector<GamepiecePose> {
  return ReadGamepieceConfig(path);
}

void WriteGamepieceConfig(const std::filesystem::path& path,
                          const std::vector<GamepiecePose>& gamepieces,
                          const FuelGenerationOptions& options) {
  ValidateEntropy(options.entropy);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write gamepiece config: " +
                             path.string());
  }
  nlohmann::json json = {{"entropy", options.entropy},
                         {"seed", options.seed},
                         {"gamepieces", nlohmann::json::array()}};
  for (const auto& gamepiece : gamepieces) {
    json["gamepieces"].push_back(PoseJson(gamepiece));
  }
  output << std::setw(2) << json << '\n';
  if (!output) {
    throw std::runtime_error("Failed while writing gamepiece config: " +
                             path.string());
  }
}

auto GenerateFuelGamepieces(const std::filesystem::path& field_directory,
                            const FuelGenerationOptions& options)
    -> std::vector<GamepiecePose> {
  ValidateEntropy(options.entropy);
  const auto config = ReadJson(field_directory / "config.json");
  const auto& gamepiece_configs = config.at("gamePieces");
  size_t fuel_index = gamepiece_configs.size();
  for (size_t index = 0; index < gamepiece_configs.size(); ++index) {
    if (gamepiece_configs.at(index).at("name").get<std::string>() == "Fuel") {
      fuel_index = index;
      break;
    }
  }
  if (fuel_index == gamepiece_configs.size()) {
    throw std::runtime_error("Field config contains no Fuel gamepiece");
  }

  const auto& fuel_config = gamepiece_configs.at(fuel_index);
  const std::string fuel_node_name = RootNodeName(
      field_directory / ("model_" + std::to_string(fuel_index) + ".glb"));
  const auto field_document = ReadGlbJson(field_directory / "model.glb");
  const size_t scene_index = field_document.value("scene", 0U);
  std::vector<Eigen::Matrix4d> staged_transforms;
  for (const auto& root :
       field_document.at("scenes").at(scene_index).at("nodes")) {
    FindNodeTransforms(field_document.at("nodes"), root.get<size_t>(),
                       Eigen::Matrix4d::Identity(), fuel_node_name,
                       staged_transforms);
  }
  const size_t expected_count = fuel_config.at("stagedObjects").size();
  if (staged_transforms.size() != expected_count) {
    throw std::runtime_error("Staged Fuel node count mismatch: expected " +
                             std::to_string(expected_count) + ", found " +
                             std::to_string(staged_transforms.size()));
  }

  const double field_length =
      config.at("widthInches").get<double>() * kInchesToMeters;
  const double field_width =
      config.at("heightInches").get<double>() * kInchesToMeters;
  const Eigen::Matrix4d staged_to_wpilib =
      FieldModelToWpilib(field_length, field_width) *
      ConfigModelTransform(config);
  const Eigen::Matrix4d piece_config_inverse =
      ConfigModelTransform(fuel_config).inverse();

  std::mt19937 random(options.seed);
  std::uniform_real_distribution<double> removal_score(0.0, 1.0);
  std::normal_distribution<double> displacement(0.0, kMaximumSpreadMeters);
  std::vector<GamepiecePose> result;
  result.reserve(staged_transforms.size());
  for (const auto& staged_transform : staged_transforms) {
    // Draw all random values for every staged piece. This makes increasing
    // entropy with the same seed move surviving pieces along stable paths.
    const double score = removal_score(random);
    const double offset_x = displacement(random);
    const double offset_y = displacement(random);
    if (score < options.entropy * kMaximumMissingFraction) {
      continue;
    }
    Eigen::Matrix4d pose_matrix =
        staged_to_wpilib * staged_transform * piece_config_inverse;
    if (options.entropy > 0.0) {
      pose_matrix(0, 3) =
          std::clamp(pose_matrix(0, 3) + options.entropy * offset_x,
                     kFuelRadiusMeters, field_length - kFuelRadiusMeters);
      pose_matrix(1, 3) =
          std::clamp(pose_matrix(1, 3) + options.entropy * offset_y,
                     kFuelRadiusMeters, field_width - kFuelRadiusMeters);
    }
    result.push_back({.type = "Fuel", .pose = frc::Pose3d(pose_matrix)});
  }
  return result;
}

}  // namespace path_camera_sim
