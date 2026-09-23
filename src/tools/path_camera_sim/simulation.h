#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "src/tools/path_camera_sim/gamepiece_config.h"

namespace path_camera_sim {

struct TrajectorySample {
  double time_seconds;
  double path_time_seconds;
  std::string path_name;
  frc::Pose2d robot_pose;
};

struct CameraCalibration {
  std::string name;
  int width;
  int height;
  Eigen::Matrix3d camera_matrix;
  cv::Mat distortion;
  frc::Transform3d camera_to_robot;
  std::filesystem::path intrinsics_path;
  std::filesystem::path extrinsics_path;
};

struct SimulationConfig {
  std::string auto_name = "Corner";
  std::string camera_name = "second_bot_left";
  std::filesystem::path repository_root;
  std::filesystem::path pathplanner_directory;
  std::filesystem::path camera_constants_path;
  std::filesystem::path field_directory;
  std::filesystem::path gamepieces_path;
  std::filesystem::path output_directory;
  double frames_per_second = 10.0;
  bool apply_distortion = true;
};

auto LoadCameraCalibration(const std::filesystem::path& constants_path,
                           const std::string& camera_name,
                           const std::filesystem::path& repository_root)
    -> CameraCalibration;
auto LoadTrajectorySamples(const std::filesystem::path& pathplanner_directory,
                           const std::string& auto_name,
                           double frames_per_second)
    -> std::vector<TrajectorySample>;

auto FieldModelToWpilib(double field_length_meters, double field_width_meters)
    -> Eigen::Matrix4d;
auto CameraWorldToOpenCv(const frc::Pose2d& robot_pose,
                         const frc::Transform3d& camera_to_robot)
    -> Eigen::Matrix4d;
auto CameraPose(const frc::Pose2d& robot_pose,
                const frc::Transform3d& camera_to_robot) -> frc::Pose3d;

void RunSimulation(int argc, const char* argv[],
                   const SimulationConfig& config);

}  // namespace path_camera_sim
