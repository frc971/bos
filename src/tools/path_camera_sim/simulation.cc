#include "src/tools/path_camera_sim/simulation.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <frc/Filesystem.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <open3d/camera/PinholeCameraParameters.h>
#include <open3d/geometry/Image.h>
#include <open3d/io/ModelIO.h>
#include <open3d/visualization/rendering/Model.h>
#include <open3d/visualization/visualizer/RenderOption.h>
#include <open3d/visualization/visualizer/Visualizer.h>

#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "src/camera/camera_constants.h"
#include "src/tools/path_camera_sim/glb_utils.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

namespace path_camera_sim {
namespace {

using open3d::visualization::rendering::TriangleMeshModel;

constexpr double kInchesToMeters = 0.0254;

auto ReadJson(const std::filesystem::path& path) -> nlohmann::json {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open JSON file: " + path.string());
  }
  nlohmann::json result;
  input >> result;
  return result;
}

auto ResolveRepositoryPath(const std::filesystem::path& configured,
                           const std::filesystem::path& repository_root)
    -> std::filesystem::path {
  if (!configured.is_absolute()) {
    return repository_root / configured;
  }
  const auto text = configured.generic_string();
  if (text == "/bos") {
    return repository_root;
  }
  if (text.starts_with("/bos/")) {
    return repository_root / text.substr(5);
  }
  return configured;
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

void TransformModel(TriangleMeshModel& model,
                    const Eigen::Matrix4d& transform) {
  for (auto& mesh : model.meshes_) {
    mesh.mesh->Transform(transform);
  }
}

auto LoadModel(const std::filesystem::path& path) -> TriangleMeshModel {
  TriangleMeshModel model;
  if (!open3d::io::ReadTriangleModel(path.string(), model)) {
    throw std::runtime_error("Open3D could not load model: " + path.string());
  }
  if (model.meshes_.empty()) {
    throw std::runtime_error("Model contains no meshes: " + path.string());
  }
  // The legacy OpenGL renderer accepts TriangleMesh rather than a complete
  // rendering model. Preserve each GLB material's base color as vertex color.
  for (auto& mesh : model.meshes_) {
    if (mesh.material_idx < model.materials_.size()) {
      mesh.mesh->PaintUniformColor(model.materials_[mesh.material_idx]
                                       .base_color.head<3>()
                                       .cast<double>());
    }
  }
  return model;
}

class TemporaryDeployDirectory {
 public:
  explicit TemporaryDeployDirectory(
      const std::filesystem::path& pathplanner_directory)
      : original_directory_(std::filesystem::current_path()) {
    const auto suffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    root_ = std::filesystem::temp_directory_path() /
            ("path_camera_sim_deploy_" + suffix);
    const auto deploy = root_ / "src/main/deploy";
    std::filesystem::create_directories(deploy);
    std::filesystem::create_directory_symlink(
        std::filesystem::absolute(pathplanner_directory),
        deploy / "pathplanner");
    std::filesystem::current_path(root_);
  }

  TemporaryDeployDirectory(const TemporaryDeployDirectory&) = delete;
  auto operator=(const TemporaryDeployDirectory&)
      -> TemporaryDeployDirectory& = delete;

  ~TemporaryDeployDirectory() {
    std::error_code error;
    std::filesystem::current_path(original_directory_, error);
    std::filesystem::remove_all(root_, error);
  }

 private:
  std::filesystem::path original_directory_;
  std::filesystem::path root_;
};

auto PoseJson(const frc::Pose3d& pose) -> nlohmann::json {
  return {
      {"translation_m", {pose.X().value(), pose.Y().value(), pose.Z().value()}},
      {"rotation_rpy_rad",
       {pose.Rotation().X().value(), pose.Rotation().Y().value(),
        pose.Rotation().Z().value()}}};
}

auto PoseJson(const frc::Pose2d& pose) -> nlohmann::json {
  return {{"translation_m", {pose.X().value(), pose.Y().value()}},
          {"rotation_rad", pose.Rotation().Radians().value()}};
}

void BuildDistortionMaps(const CameraCalibration& calibration, cv::Mat& map_x,
                         cv::Mat& map_y) {
  cv::Mat camera_matrix(3, 3, CV_64F,
                        const_cast<double*>(calibration.camera_matrix.data()));
  camera_matrix = camera_matrix.clone();
  // Eigen is column-major while OpenCV is row-major.
  cv::transpose(camera_matrix, camera_matrix);

  std::vector<cv::Point2f> distorted_pixels;
  distorted_pixels.reserve(calibration.width * calibration.height);
  for (int y = 0; y < calibration.height; ++y) {
    for (int x = 0; x < calibration.width; ++x) {
      distorted_pixels.emplace_back(static_cast<float>(x),
                                    static_cast<float>(y));
    }
  }
  std::vector<cv::Point2f> undistorted_pixels;
  cv::undistortPoints(distorted_pixels, undistorted_pixels, camera_matrix,
                      calibration.distortion, cv::noArray(), camera_matrix);
  map_x.create(calibration.height, calibration.width, CV_32FC1);
  map_y.create(calibration.height, calibration.width, CV_32FC1);
  for (int y = 0; y < calibration.height; ++y) {
    for (int x = 0; x < calibration.width; ++x) {
      const auto& source = undistorted_pixels[y * calibration.width + x];
      map_x.at<float>(y, x) = source.x;
      map_y.at<float>(y, x) = source.y;
    }
  }
}

auto ImageToBgr(const open3d::geometry::Image& image) -> cv::Mat {
  if (image.num_of_channels_ != 3 && image.num_of_channels_ != 4) {
    throw std::runtime_error("Open3D returned an unsupported image format");
  }
  const int channels = image.num_of_channels_;
  int type;
  if (image.bytes_per_channel_ == 1) {
    type = channels == 3 ? CV_8UC3 : CV_8UC4;
  } else if (image.bytes_per_channel_ == 4) {
    type = channels == 3 ? CV_32FC3 : CV_32FC4;
  } else {
    throw std::runtime_error("Open3D returned an unsupported image bit depth");
  }
  cv::Mat source(image.height_, image.width_, type,
                 const_cast<uint8_t*>(image.data_.data()));
  cv::Mat byte_source;
  if (image.bytes_per_channel_ == 4) {
    source.convertTo(byte_source, channels == 3 ? CV_8UC3 : CV_8UC4, 255.0);
  } else {
    byte_source = source;
  }
  cv::Mat bgr;
  cv::cvtColor(byte_source, bgr,
               channels == 3 ? cv::COLOR_RGB2BGR : cv::COLOR_RGBA2BGR);
  return bgr;
}

auto CachedFieldPath(const SimulationConfig& config) -> std::filesystem::path {
  return config.output_directory / ".cache" / "field_without_staged.glb";
}

void RemoveStaleFrames(const std::filesystem::path& output_directory,
                       size_t frame_count) {
  for (const auto& entry :
       std::filesystem::directory_iterator(output_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    const bool frame_name =
        name.size() == 16 && name.starts_with("frame_") &&
        name.ends_with(".png") &&
        std::all_of(name.begin() + 6, name.begin() + 12,
                    [](unsigned char value) { return std::isdigit(value); });
    if (frame_name &&
        static_cast<size_t>(std::stoul(name.substr(6, 6))) >= frame_count) {
      std::filesystem::remove(entry.path());
    }
  }
}

auto PrepareFieldModel(const SimulationConfig& config,
                       const nlohmann::json& field_config)
    -> TriangleMeshModel {
  const auto source = config.field_directory / "model.glb";
  const auto cleaned = CachedFieldPath(config);
  const auto config_path = config.field_directory / "config.json";
  const bool cache_fresh =
      std::filesystem::exists(cleaned) &&
      std::filesystem::last_write_time(cleaned) >=
          std::max(std::filesystem::last_write_time(source),
                   std::filesystem::last_write_time(config_path));
  if (!cache_fresh) {
    const auto result = PruneStagedGamepieces(source, config_path,
                                              config.field_directory, cleaned);
    std::cout << "Removed " << result.removed_nodes
              << " staged gamepiece meshes from the cached field model\n";
  }
  auto field = LoadModel(cleaned);
  const double length =
      field_config.at("widthInches").get<double>() * kInchesToMeters;
  const double width =
      field_config.at("heightInches").get<double>() * kInchesToMeters;
  TransformModel(field, FieldModelToWpilib(length, width) *
                            ConfigModelTransform(field_config));
  return field;
}

}  // namespace

auto LoadCameraCalibration(const std::filesystem::path& constants_path,
                           const std::string& camera_name,
                           const std::filesystem::path& repository_root)
    -> CameraCalibration {
  const auto constants = camera::GetCameraConstants(constants_path.string());
  const auto iterator = constants.find(camera_name);
  if (iterator == constants.end()) {
    throw std::runtime_error("Camera '" + camera_name + "' is not present in " +
                             constants_path.string());
  }
  const auto& camera = iterator->second;
  if (!camera.intrinsics_path || !camera.extrinsics_path ||
      !camera.frame_width || !camera.frame_height) {
    throw std::runtime_error("Camera '" + camera_name +
                             "' lacks intrinsics, extrinsics, or frame size");
  }
  const auto intrinsics =
      ResolveRepositoryPath(*camera.intrinsics_path, repository_root);
  const auto extrinsics =
      ResolveRepositoryPath(*camera.extrinsics_path, repository_root);
  const auto intrinsics_json = utils::ReadIntrinsics(intrinsics.string());
  return {.name = camera_name,
          .width = static_cast<int>(*camera.frame_width),
          .height = static_cast<int>(*camera.frame_height),
          .camera_matrix =
              utils::CameraMatrixFromJson<Eigen::Matrix3d>(intrinsics_json),
          .distortion =
              utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics_json),
          .camera_to_robot = utils::ExtrinsicsJsonToCameraToRobot(
              utils::ReadExtrinsics(extrinsics.string())),
          .intrinsics_path = intrinsics,
          .extrinsics_path = extrinsics};
}

auto LoadTrajectorySamples(const std::filesystem::path& pathplanner_directory,
                           const std::string& auto_name,
                           double frames_per_second)
    -> std::vector<TrajectorySample> {
  if (!(frames_per_second > 0.0) || !std::isfinite(frames_per_second)) {
    throw std::runtime_error("FPS must be a finite positive number");
  }
  if (!std::filesystem::exists(pathplanner_directory / "autos" /
                               (auto_name + ".auto"))) {
    throw std::runtime_error(
        "PathPlanner auto does not exist: " +
        (pathplanner_directory / "autos" / (auto_name + ".auto")).string());
  }

  TemporaryDeployDirectory deploy(pathplanner_directory);
  const auto robot_config = pathplanner::RobotConfig::fromGUISettings();
  const auto paths =
      pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auto_name);
  if (paths.empty()) {
    throw std::runtime_error("Auto '" + auto_name + "' contains no paths");
  }

  const double step = 1.0 / frames_per_second;
  double global_offset = 0.0;
  std::vector<TrajectorySample> samples;
  for (size_t path_index = 0; path_index < paths.size(); ++path_index) {
    auto trajectory = paths[path_index]->getIdealTrajectory(robot_config);
    if (!trajectory) {
      throw std::runtime_error("Path '" + paths[path_index]->name +
                               "' has no ideal starting state");
    }
    const double duration = trajectory->getTotalTime().value();
    const size_t regular_count =
        static_cast<size_t>(std::floor(duration / step));
    const size_t first_index = path_index == 0 ? 0 : 1;
    for (size_t i = first_index; i <= regular_count; ++i) {
      const double local_time = std::min(i * step, duration);
      samples.push_back(
          {.time_seconds = global_offset + local_time,
           .path_time_seconds = local_time,
           .path_name = paths[path_index]->name,
           .robot_pose = trajectory->sample(units::second_t{local_time}).pose});
    }
    const double last_regular = regular_count * step;
    if (duration - last_regular > 1e-9) {
      samples.push_back(
          {.time_seconds = global_offset + duration,
           .path_time_seconds = duration,
           .path_name = paths[path_index]->name,
           .robot_pose = trajectory->sample(units::second_t{duration}).pose});
    }
    global_offset += duration;
  }
  return samples;
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

auto CameraPose(const frc::Pose2d& robot_pose,
                const frc::Transform3d& camera_to_robot) -> frc::Pose3d {
  return frc::Pose3d(robot_pose).TransformBy(camera_to_robot.Inverse());
}

auto CameraWorldToOpenCv(const frc::Pose2d& robot_pose,
                         const frc::Transform3d& camera_to_robot)
    -> Eigen::Matrix4d {
  Eigen::Matrix4d wpilib_camera_to_opencv = Eigen::Matrix4d::Zero();
  wpilib_camera_to_opencv(0, 1) = -1.0;  // WPILib left -> OpenCV right.
  wpilib_camera_to_opencv(1, 2) = -1.0;  // WPILib up -> OpenCV down.
  wpilib_camera_to_opencv(2, 0) = 1.0;   // WPILib forward -> OpenCV forward.
  wpilib_camera_to_opencv(3, 3) = 1.0;
  return wpilib_camera_to_opencv *
         CameraPose(robot_pose, camera_to_robot).ToMatrix().inverse();
}

void RunSimulation(int argc, const char* argv[],
                   const SimulationConfig& config) {
  (void)argc;
  (void)argv;
  if (!std::filesystem::exists(config.field_directory / "model.glb") ||
      !std::filesystem::exists(config.field_directory / "config.json")) {
    throw std::runtime_error(
        "Field directory must contain model.glb and config.json: " +
        config.field_directory.string());
  }
  std::filesystem::create_directories(config.output_directory);
  const auto field_config = ReadJson(config.field_directory / "config.json");
  auto field = PrepareFieldModel(config, field_config);
  const auto pieces = LoadGamepieces(config.gamepieces_path);
  const auto samples = LoadTrajectorySamples(
      config.pathplanner_directory, config.auto_name, config.frames_per_second);
  const auto calibration = LoadCameraCalibration(
      config.camera_constants_path, config.camera_name, config.repository_root);

  open3d::visualization::Visualizer visualizer;
  if (!visualizer.CreateVisualizerWindow("Path camera simulator",
                                         calibration.width, calibration.height,
                                         0, 0, false)) {
    throw std::runtime_error(
        "Open3D could not create an OpenGL window; check DISPLAY and graphics "
        "access");
  }
  visualizer.GetRenderOption().background_color_ =
      Eigen::Vector3d(0.08, 0.08, 0.10);
  visualizer.GetRenderOption().light_on_ = true;
  for (const auto& mesh : field.meshes_) {
    if (!visualizer.AddGeometry(mesh.mesh, true)) {
      throw std::runtime_error(
          "Open3D could not add a field mesh to the scene");
    }
  }

  std::unordered_map<std::string, size_t> piece_model_index;
  const auto& piece_configs = field_config.at("gamePieces");
  for (size_t i = 0; i < piece_configs.size(); ++i) {
    piece_model_index.emplace(piece_configs.at(i).at("name").get<std::string>(),
                              i);
  }
  std::vector<TriangleMeshModel> piece_models;
  piece_models.reserve(pieces.size());
  for (size_t i = 0; i < pieces.size(); ++i) {
    const auto config_index = piece_model_index.find(pieces[i].type);
    if (config_index == piece_model_index.end()) {
      throw std::runtime_error("No field gamepiece model named '" +
                               pieces[i].type + "'");
    }
    const size_t model_index = config_index->second;
    auto model = LoadModel(config.field_directory /
                           ("model_" + std::to_string(model_index) + ".glb"));
    TransformModel(model,
                   pieces[i].pose.ToMatrix() *
                       ConfigModelTransform(piece_configs.at(model_index)));
    for (const auto& mesh : model.meshes_) {
      if (!visualizer.AddGeometry(mesh.mesh, true)) {
        throw std::runtime_error(
            "Open3D could not add a gamepiece mesh to the scene");
      }
    }
    piece_models.emplace_back(std::move(model));
  }

  open3d::camera::PinholeCameraParameters camera_parameters;
  camera_parameters.intrinsic_.SetIntrinsics(
      calibration.width, calibration.height, calibration.camera_matrix(0, 0),
      calibration.camera_matrix(1, 1), calibration.camera_matrix(0, 2),
      calibration.camera_matrix(1, 2));
  cv::Mat map_x;
  cv::Mat map_y;
  if (config.apply_distortion) {
    BuildDistortionMaps(calibration, map_x, map_y);
  }

  nlohmann::json manifest = {
      {"auto", config.auto_name},
      {"camera", calibration.name},
      {"fps", config.frames_per_second},
      {"resolution", {calibration.width, calibration.height}},
      {"apply_distortion", config.apply_distortion},
      {"intrinsics_path", calibration.intrinsics_path.string()},
      {"extrinsics_path", calibration.extrinsics_path.string()},
      {"camera_matrix",
       {{calibration.camera_matrix(0, 0), calibration.camera_matrix(0, 1),
         calibration.camera_matrix(0, 2)},
        {calibration.camera_matrix(1, 0), calibration.camera_matrix(1, 1),
         calibration.camera_matrix(1, 2)},
        {calibration.camera_matrix(2, 0), calibration.camera_matrix(2, 1),
         calibration.camera_matrix(2, 2)}}},
      {"distortion_coefficients",
       {calibration.distortion.at<double>(0, 0),
        calibration.distortion.at<double>(0, 1),
        calibration.distortion.at<double>(0, 2),
        calibration.distortion.at<double>(0, 3),
        calibration.distortion.at<double>(0, 4)}},
      {"gamepieces", nlohmann::json::array()},
      {"frames", nlohmann::json::array()}};
  for (const auto& piece : pieces) {
    auto item = PoseJson(piece.pose);
    item["type"] = piece.type;
    manifest["gamepieces"].push_back(std::move(item));
  }

  for (size_t i = 0; i < samples.size(); ++i) {
    const auto camera_pose =
        CameraPose(samples[i].robot_pose, calibration.camera_to_robot);
    camera_parameters.extrinsic_ =
        CameraWorldToOpenCv(samples[i].robot_pose, calibration.camera_to_robot);
    if (!visualizer.GetViewControl().ConvertFromPinholeCameraParameters(
            camera_parameters, true)) {
      throw std::runtime_error("Open3D rejected calibrated camera parameters");
    }
    visualizer.PollEvents();
    const auto image = visualizer.CaptureScreenFloatBuffer(true);
    cv::Mat output = ImageToBgr(*image);
    if (config.apply_distortion) {
      cv::Mat distorted;
      cv::remap(output, distorted, map_x, map_y, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
      output = std::move(distorted);
    }
    std::ostringstream filename;
    filename << "frame_" << std::setfill('0') << std::setw(6) << i << ".png";
    const auto frame_path = config.output_directory / filename.str();
    if (!cv::imwrite(frame_path.string(), output)) {
      throw std::runtime_error("Failed to write frame: " + frame_path.string());
    }
    manifest["frames"].push_back(
        {{"file", filename.str()},
         {"time_s", samples[i].time_seconds},
         {"path_time_s", samples[i].path_time_seconds},
         {"path", samples[i].path_name},
         {"robot_pose", PoseJson(samples[i].robot_pose)},
         {"camera_pose", PoseJson(camera_pose)}});
    if (i % 25 == 0 || i + 1 == samples.size()) {
      std::cout << "Rendered frame " << (i + 1) << '/' << samples.size()
                << '\n';
    }
  }

  RemoveStaleFrames(config.output_directory, samples.size());

  std::ofstream manifest_output(config.output_directory / "manifest.json");
  if (!manifest_output) {
    throw std::runtime_error("Unable to write output manifest");
  }
  manifest_output << std::setw(2) << manifest << '\n';

  visualizer.DestroyVisualizerWindow();
}

}  // namespace path_camera_sim
