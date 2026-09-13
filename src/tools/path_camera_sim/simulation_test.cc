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

}  // namespace
}  // namespace path_camera_sim
