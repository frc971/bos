#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "src/camera/camera_constants.h"
#include "src/gamepiece/lane_density.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

ABSL_FLAG(std::string, image_path, "", "Path to the test image");  //NOLINT
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,   //NOLINT
          "Camera name (for intrinsics)");
ABSL_FLAG(std::string, output_folder, "lane_density_output",
          "Folder for annotated output images");
ABSL_FLAG(std::string, field_image, "constants/misc/2026field.png",
          "Path to the field image");

namespace {

constexpr int kFieldImageCrop = 270;
constexpr double kFieldLengthMeters = 16.46;
constexpr double kFieldWidthMeters = 8.23;

auto DrawLaneDensity(std::string_view label, const cv::Scalar& color,
                     cv::Mat& image, cv::Point origin) -> void {
  cv::putText(image, std::string(label), origin, cv::FONT_HERSHEY_SIMPLEX, 0.7,
              color, 2, cv::LINE_AA);
}

auto DrawRobotPose(cv::Mat& field_image, const frc::Pose3d& robot_pose)
    -> void {
  const cv::Point robot_position(
      cvRound(field_image.cols * (robot_pose.X().value() / kFieldLengthMeters)),
      cvRound(field_image.rows * (robot_pose.Y().value() / kFieldWidthMeters)));
  const double heading = robot_pose.ToPose2d().Rotation().Radians().value();
  constexpr double kRobotArrowLength = 50.0;
  const cv::Point arrow_end(
      cvRound(robot_position.x + kRobotArrowLength * std::cos(heading)),
      cvRound(robot_position.y + kRobotArrowLength * std::sin(heading)));

  cv::circle(field_image, robot_position, 12, cv::Scalar(0, 0, 255), -1,
             cv::LINE_AA);
  cv::arrowedLine(field_image, robot_position, arrow_end, cv::Scalar(0, 0, 255),
                  5, cv::LINE_AA, 0, 0.5);
}

auto AnnotateImage(const cv::Mat& image, const cv::Mat& undistorted_image,
                   gamepiece::LaneDensityTracker& tracker,
                   const frc::Pose3d& robot_pose) -> cv::Mat {
  cv::Mat annotated = undistorted_image.clone();
  const auto lane_boundaries = tracker.GetImageLaneBoundaries(robot_pose);
  const auto densities = tracker.GetLaneDensities(image, robot_pose);

  for (size_t boundary_index = 0; boundary_index < lane_boundaries.size();
       ++boundary_index) {
    const auto& boundary = lane_boundaries[boundary_index];
    if (!boundary.valid || !std::isfinite(boundary.origin[0]) ||
        !std::isfinite(boundary.origin[1]) || !std::isfinite(boundary.end[0]) ||
        !std::isfinite(boundary.end[1])) {
      continue;
    }

    cv::Point origin(cvRound(boundary.origin[0]), cvRound(boundary.origin[1]));
    cv::Point end(cvRound(boundary.end[0]), cvRound(boundary.end[1]));
    if (cv::clipLine(annotated.size(), origin, end)) {
      cv::line(annotated, origin, end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
  }

  for (size_t lane_index = 0; lane_index < densities.size(); ++lane_index) {
    std::ostringstream label;
    label << "Lane " << lane_index << " density: " << std::fixed
          << std::setprecision(6) << densities[lane_index];
    DrawLaneDensity(label.str(), cv::Scalar(0, 255, 0), annotated,
                    cv::Point(20, 35 + static_cast<int>(lane_index) * 30));
  }
  return annotated;
}

auto AnnotateField(const cv::Mat& field_image, const frc::Pose3d& robot_pose)
    -> cv::Mat {
  cv::Mat annotated_field = field_image.clone();
  DrawRobotPose(annotated_field, robot_pose);
  return annotated_field;
}

}  // namespace

auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);

  const std::filesystem::path image_path(absl::GetFlag(FLAGS_image_path));
  if (image_path.empty() || !std::filesystem::is_regular_file(image_path)) {
    LOG(FATAL) << "Image path is empty or does not exist: " << image_path;
  }

  const std::filesystem::path output_folder_path(
      absl::GetFlag(FLAGS_output_folder));
  if (output_folder_path.empty()) {
    LOG(FATAL) << "Output folder must not be empty";
  }
  std::error_code output_folder_error;
  std::filesystem::create_directories(output_folder_path, output_folder_error);
  if (output_folder_error ||
      !std::filesystem::is_directory(output_folder_path)) {
    LOG(FATAL) << "Unable to create output folder: " << output_folder_path
               << (output_folder_error ? ": " + output_folder_error.message()
                                       : "");
  }

  const auto camera_name = absl::GetFlag(FLAGS_camera_name);
  if (!camera_name.has_value()) {
    LOG(FATAL) << "--camera_name is required so lane-density calibration can "
                  "be loaded";
  }
  const auto camera_constants = camera::GetCameraConstants();
  if (!camera_constants.contains(*camera_name)) {
    LOG(FATAL) << "Unknown camera name: " << *camera_name;
  }

  gamepiece::LaneDensityTracker tracker(camera_constants.at(*camera_name));

  const std::filesystem::path field_image_path(
      absl::GetFlag(FLAGS_field_image));
  cv::Mat field_image = cv::imread(field_image_path.string(), cv::IMREAD_COLOR);
  if (field_image.empty()) {
    LOG(FATAL) << "Unable to read field image: " << field_image_path;
  }
  if (field_image.cols <= 2 * kFieldImageCrop) {
    LOG(FATAL) << "Field image is too narrow to crop: " << field_image_path;
  }
  field_image = field_image(cv::Rect(kFieldImageCrop, 0,
                                     field_image.cols - 2 * kFieldImageCrop,
                                     field_image.rows))
                    .clone();

  cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    LOG(FATAL) << "Unable to read image: " << image_path;
  }
  const nlohmann::json intrinsics = utils::ReadIntrinsics(
      camera_constants.at(*camera_name).intrinsics_path.value());
  const cv::Mat camera_matrix =
      utils::CameraMatrixFromJson<cv::Mat>(intrinsics);
  const cv::Mat distortion_coefficients =
      utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics);
  localization::OpenCVAprilTagDetector detector(intrinsics);
  localization::MultiTagSolver solver(camera_constants.at(*camera_name));

  cv::Mat grayscale;
  cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
  camera::timestamped_frame_t timestamped{
      .frame = grayscale, .timestamp = 0.0, .invalid = false};
  const auto detections = detector.GetTagDetections(timestamped);
  const auto position_estimates = solver.EstimatePosition(detections);
  if (position_estimates.empty()) {
    LOG(ERROR) << "Unable to estimate robot pose from " << detections.size()
               << " detected AprilTags in: " << image_path;
    return 1;
  }
  const frc::Pose3d& robot_pose = position_estimates.front().pose;
  cv::Mat undistorted_image;
  cv::undistort(image, undistorted_image, camera_matrix,
                distortion_coefficients, camera_matrix);
  cv::Mat annotated =
      AnnotateImage(image, undistorted_image, tracker, robot_pose);
  const std::filesystem::path output_path =
      output_folder_path / image_path.filename();
  if (!cv::imwrite(output_path.string(), annotated)) {
    LOG(FATAL) << "Unable to write annotated image: " << output_path;
  }

  cv::Mat annotated_field = AnnotateField(field_image, robot_pose);
  LOG(INFO) << "ROBOT POSE";
  utils::PrintPose3d(robot_pose);
  const std::filesystem::path field_output_path =
      output_folder_path / ("field_" + image_path.filename().string());
  if (!cv::imwrite(field_output_path.string(), annotated_field)) {
    LOG(FATAL) << "Unable to write annotated field image: "
               << field_output_path;
  }
  LOG(INFO) << "Processed image: " << image_path << " -> " << output_path;

  return 0;
}
