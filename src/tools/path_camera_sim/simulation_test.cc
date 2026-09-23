#include "src/tools/path_camera_sim/simulation.h"

#include <filesystem>

#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>

#include "src/tools/path_camera_sim/glb_utils.h"

namespace path_camera_sim {
namespace {

TEST(CoordinateTransforms, FieldModelUsesBlueWallOrigin) {
  const Eigen::Matrix4d transform = FieldModelToWpilib(16.0, 8.0);
  const Eigen::Vector4d model_blue_wall_center(8.0, 4.0, 0.0, 1.0);
  const Eigen::Vector4d model_red_wall_center(-8.0, 4.0, 0.0, 1.0);
  EXPECT_TRUE((transform * model_blue_wall_center)
                  .isApprox(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)));
  EXPECT_TRUE((transform * model_red_wall_center)
                  .isApprox(Eigen::Vector4d(16.0, 0.0, 0.0, 1.0)));
}

TEST(CoordinateTransforms, CameraAxesBecomeOpenCvAxes) {
  const auto world_to_camera =
      CameraWorldToOpenCv(frc::Pose2d(), frc::Transform3d());
  EXPECT_TRUE((world_to_camera * Eigen::Vector4d(1.0, 0.0, 0.0, 1.0))
                  .isApprox(Eigen::Vector4d(0.0, 0.0, 1.0, 1.0)));
  EXPECT_TRUE((world_to_camera * Eigen::Vector4d(0.0, 1.0, 0.0, 1.0))
                  .isApprox(Eigen::Vector4d(-1.0, 0.0, 0.0, 1.0)));
  EXPECT_TRUE((world_to_camera * Eigen::Vector4d(0.0, 0.0, 1.0, 1.0))
                  .isApprox(Eigen::Vector4d(0.0, -1.0, 0.0, 1.0)));
}

TEST(Inputs, LoadsCornerAutoAtTenFps) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const auto samples =
      LoadTrajectorySamples(root / "paths/pathplanner", "Corner", 10.0);
  ASSERT_GT(samples.size(), 10U);
  EXPECT_EQ(samples.front().path_name, "Corner Start");
  EXPECT_EQ(samples.back().path_name, "Corner");
  EXPECT_NEAR(samples.front().robot_pose.X().value(), 3.4874, 0.02);
  EXPECT_NEAR(samples.back().robot_pose.X().value(), 0.5749, 0.02);
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GT(samples[i].time_seconds, samples[i - 1].time_seconds);
  }
}

TEST(Inputs, LoadsExistingCameraCalibration) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const auto calibration = LoadCameraCalibration(
      root / "constants/camera_constants.json", "second_bot_left", root);
  EXPECT_EQ(calibration.width, 1280);
  EXPECT_EQ(calibration.height, 800);
  EXPECT_NEAR(calibration.camera_matrix(0, 0), 905.4343, 1e-3);
  EXPECT_NEAR(calibration.camera_matrix(1, 1), 905.0218, 1e-3);
  EXPECT_TRUE(std::filesystem::exists(calibration.intrinsics_path));
  EXPECT_TRUE(std::filesystem::exists(calibration.extrinsics_path));
}

TEST(FieldAssets, RemovesDeclaredStagedGamepieceMeshes) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const auto temp = std::filesystem::temp_directory_path() /
                    "path_camera_sim_pruned_test.glb";
  const auto result = PruneStagedGamepieces(root / "field-cad/model.glb",
                                            root / "field-cad/config.json",
                                            root / "field-cad", temp);
  EXPECT_EQ(result.expected_nodes, 456U);
  EXPECT_EQ(result.removed_nodes, 456U);
  std::error_code error;
  std::filesystem::remove(temp, error);
}

TEST(GamepieceConfig, ZeroEntropyReproducesEveryStagedFuel) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const auto first = GenerateFuelGamepieces(
      root / "field-cad", FuelGenerationOptions{.entropy = 0.0, .seed = 1});
  const auto second = GenerateFuelGamepieces(
      root / "field-cad", FuelGenerationOptions{.entropy = 0.0, .seed = 99});
  ASSERT_EQ(first.size(), 456U);
  ASSERT_EQ(second.size(), first.size());
  EXPECT_NEAR(first.front().pose.X().value(), 0.22856825, 1e-9);
  EXPECT_NEAR(first.front().pose.Y().value(), 6.0410979, 1e-9);
  EXPECT_NEAR(first.front().pose.Z().value(), 0.075, 1e-9);
  for (size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].type, "Fuel");
    EXPECT_TRUE(first[index].pose.ToMatrix().isApprox(
        second[index].pose.ToMatrix(), 1e-12));
  }
}

TEST(GamepieceConfig, EntropySpreadsAndRemovesFuelDeterministically) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const FuelGenerationOptions options{.entropy = 1.0, .seed = 42};
  const auto staged = GenerateFuelGamepieces(
      root / "field-cad", FuelGenerationOptions{.entropy = 0.0, .seed = 42});
  const auto scattered = GenerateFuelGamepieces(root / "field-cad", options);
  const auto repeated = GenerateFuelGamepieces(root / "field-cad", options);
  EXPECT_LT(scattered.size(), staged.size());
  EXPECT_GT(scattered.size(), staged.size() / 3);
  ASSERT_EQ(repeated.size(), scattered.size());
  for (size_t index = 0; index < scattered.size(); ++index) {
    EXPECT_TRUE(scattered[index].pose.ToMatrix().isApprox(
        repeated[index].pose.ToMatrix(), 1e-12));
    EXPECT_GE(scattered[index].pose.X().value(), 0.075);
    EXPECT_LE(scattered[index].pose.X().value(), 16.541);
    EXPECT_GE(scattered[index].pose.Y().value(), 0.075);
    EXPECT_LE(scattered[index].pose.Y().value(), 7.994);
  }
}

TEST(GamepieceConfig, WrittenConfigRoundTripsThroughReader) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  const FuelGenerationOptions options{.entropy = 0.4, .seed = 7};
  const auto generated = GenerateFuelGamepieces(root / "field-cad", options);
  const auto path = std::filesystem::temp_directory_path() /
                    "fuel_gamepiece_config_round_trip.json";
  WriteGamepieceConfig(path, generated, options);
  const auto loaded = ReadGamepieceConfig(path);
  ASSERT_EQ(loaded.size(), generated.size());
  for (size_t index = 0; index < loaded.size(); ++index) {
    EXPECT_EQ(loaded[index].type, generated[index].type);
    EXPECT_TRUE(loaded[index].pose.ToMatrix().isApprox(
        generated[index].pose.ToMatrix(), 1e-9));
  }
  std::error_code error;
  std::filesystem::remove(path, error);
}

TEST(GamepieceConfig, RejectsEntropyOutsideNormalizedRange) {
  const auto root = std::filesystem::path(BOS_SOURCE_DIR);
  EXPECT_THROW(GenerateFuelGamepieces(
                   root / "field-cad",
                   FuelGenerationOptions{.entropy = -0.01, .seed = 0}),
               std::runtime_error);
  EXPECT_THROW(
      GenerateFuelGamepieces(root / "field-cad",
                             FuelGenerationOptions{.entropy = 1.01, .seed = 0}),
      std::runtime_error);
}

}  // namespace
}  // namespace path_camera_sim
