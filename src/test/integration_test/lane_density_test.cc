#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "src/camera/camera_constants.h"
#include "src/gamepiece/gamepiece.h"
#include "src/gamepiece/lane_density.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/image_utils.h"

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
constexpr size_t kLaneCount = 4;

struct LaneRegion {
  std::vector<cv::Point> polygon;
  float area = 0.0F;
  bool valid = false;
};

using LaneRegions = std::array<LaneRegion, kLaneCount>;

const std::array<cv::Scalar, kLaneCount> kLaneColors{
    cv::Scalar(255, 80, 80), cv::Scalar(80, 255, 80), cv::Scalar(80, 80, 255),
    cv::Scalar(255, 180, 40)};

auto WriteImage(const std::filesystem::path& path, const cv::Mat& image)
    -> void {
  if (!cv::imwrite(path.string(), image)) {
    LOG(FATAL) << "Unable to write debug image: " << path;
  }
}

auto ClipLaneBoundariesToImage(
    std::array<gamepiece::LaneDensityTracker::image_lane_segment_t, 2 * 2 + 1>
        lane_boundaries,
    const cv::Size image_size)
    -> std::array<gamepiece::LaneDensityTracker::image_lane_segment_t,
                  2 * 2 + 1> {
  for (auto& boundary : lane_boundaries) {
    if (!boundary.valid || !std::isfinite(boundary.origin[0]) ||
        !std::isfinite(boundary.origin[1]) || !std::isfinite(boundary.end[0]) ||
        !std::isfinite(boundary.end[1])) {
      boundary.valid = false;
      continue;
    }
    cv::Point origin(cvRound(boundary.origin[0]), cvRound(boundary.origin[1]));
    cv::Point end(cvRound(boundary.end[0]), cvRound(boundary.end[1]));
    boundary.valid = cv::clipLine(image_size, origin, end);
    boundary.origin =
        cv::Vec2f{static_cast<float>(origin.x), static_cast<float>(origin.y)};
    boundary.end =
        cv::Vec2f{static_cast<float>(end.x), static_cast<float>(end.y)};
  }
  return lane_boundaries;
}

auto CalculateLaneRegions(
    const std::array<gamepiece::LaneDensityTracker::image_lane_segment_t,
                     2 * 2 + 1>& lane_boundaries) -> LaneRegions {
  LaneRegions regions{};
  std::optional<std::pair<cv::Vec2f, cv::Vec2f>> previous_lane;

  for (size_t boundary_index = 0; boundary_index < lane_boundaries.size();
       ++boundary_index) {
    const auto& boundary = lane_boundaries[boundary_index];
    if (!boundary.valid) {
      if (boundary_index != 0) {
        regions[boundary_index - 1].area = 0.0F;
      }
      previous_lane = std::nullopt;
      continue;
    }

    const std::pair<cv::Vec2f, cv::Vec2f> current_lane{boundary.origin,
                                                       boundary.end};

    if (boundary_index != 0) {
      if (!previous_lane.has_value()) {
        regions[boundary_index - 1].area = 0.0F;
        previous_lane = current_lane;
        continue;
      }

      cv::Vec2f diagonal = current_lane.second - previous_lane->first;
      const float diagonal_length = cv::norm(diagonal);
      if (diagonal_length == 0.0F) {
        regions[boundary_index - 1].area = 0.0F;
        previous_lane = current_lane;
        continue;
      }
      diagonal /= diagonal_length;
      const cv::Vec2f offset_1 = current_lane.first - previous_lane->first;
      const cv::Vec2f perpendicular_component_1 =
          offset_1 - diagonal.dot(offset_1) * diagonal;
      const cv::Vec2f offset_2 = previous_lane->second - previous_lane->first;
      const cv::Vec2f perpendicular_component_2 =
          offset_2 - diagonal.dot(offset_2) * diagonal;
      const float area = 0.5F * diagonal_length *
                         (cv::norm(perpendicular_component_1) +
                          cv::norm(perpendicular_component_2));

      if (area > 0.0F) {
        LaneRegion& region = regions[boundary_index - 1];
        region.area = area;
        region.valid = true;
        region.polygon = {cv::Point(cvRound(previous_lane->first[0]),
                                    cvRound(previous_lane->first[1])),
                          cv::Point(cvRound(current_lane.first[0]),
                                    cvRound(current_lane.first[1])),
                          cv::Point(cvRound(current_lane.second[0]),
                                    cvRound(current_lane.second[1])),
                          cv::Point(cvRound(previous_lane->second[0]),
                                    cvRound(previous_lane->second[1]))};
      }
    }
    previous_lane = current_lane;
  }
  return regions;
}

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

auto AnnotateImage(
    const cv::Mat& image,
    const std::array<gamepiece::LaneDensityTracker::image_lane_segment_t,
                     2 * 2 + 1>& lane_boundaries,
    const std::array<float, kLaneCount>& densities,
    const LaneRegions& lane_regions) -> cv::Mat {
  cv::Mat annotated = image.clone();

  for (size_t lane_index = 0; lane_index < lane_regions.size(); ++lane_index) {
    const LaneRegion& region = lane_regions[lane_index];
    if (!region.valid) {
      continue;
    }
    cv::Mat overlay = annotated.clone();
    const std::vector<std::vector<cv::Point>> polygons{region.polygon};
    cv::fillPoly(overlay, polygons, kLaneColors[lane_index]);
    cv::addWeighted(overlay, 0.25, annotated, 0.75, 0.0, annotated);
    cv::polylines(annotated, region.polygon, true, kLaneColors[lane_index], 2,
                  cv::LINE_AA);
  }

  for (size_t boundary_index = 0; boundary_index < lane_boundaries.size();
       ++boundary_index) {
    const auto& boundary = lane_boundaries[boundary_index];
    if (!boundary.valid || !std::isfinite(boundary.origin[0]) ||
        !std::isfinite(boundary.origin[1]) || !std::isfinite(boundary.end[0]) ||
        !std::isfinite(boundary.end[1])) {
      continue;
    }

    const cv::Point origin(cvRound(boundary.origin[0]),
                           cvRound(boundary.origin[1]));
    const cv::Point end(cvRound(boundary.end[0]), cvRound(boundary.end[1]));
    cv::line(annotated, origin, end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  }

  for (size_t lane_index = 0; lane_index < densities.size(); ++lane_index) {
    std::ostringstream label;
    label << "Lane " << lane_index << " density: " << std::fixed
          << std::setprecision(6) << densities[lane_index]
          << " area: " << std::setprecision(1) << lane_regions[lane_index].area
          << " px^2";
    DrawLaneDensity(label.str(), kLaneColors[lane_index], annotated,
                    cv::Point(20, 35 + static_cast<int>(lane_index) * 35));
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
  cv::Mat undistorted_image;
  cv::undistort(image, undistorted_image, camera_matrix,
                distortion_coefficients);
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
  std::vector<cv::Point2f> thresholded_points;
  cv::Mat3b hsv_image;
  cv::Mat1b threshold_mask;
  utils::HSVThreshold(image, gamepiece::hsv_color_range.first,
                      gamepiece::hsv_color_range.second,
                      gamepiece::minimum_saturation, thresholded_points,
                      hsv_image, threshold_mask);
  cv::Mat hsv_color_view;
  cv::cvtColor(hsv_image, hsv_color_view, cv::COLOR_HSV2BGR);
  cv::Mat thresholded_bgr;
  cv::cvtColor(threshold_mask, thresholded_bgr, cv::COLOR_GRAY2BGR);
  cv::Mat thresholded_color;
  cv::bitwise_and(image, image, thresholded_color, threshold_mask);
  const std::filesystem::path hsv_output_path =
      output_folder_path /
      ("hsv_color_view_" + image_path.stem().string() + ".png");
  WriteImage(hsv_output_path, hsv_color_view);
  const std::filesystem::path threshold_output_path =
      output_folder_path / ("threshold_" + image_path.stem().string() + ".png");
  WriteImage(threshold_output_path, thresholded_bgr);
  const std::filesystem::path thresholded_color_output_path =
      output_folder_path /
      ("hsv_thresholded_color_" + image_path.stem().string() + ".png");
  WriteImage(thresholded_color_output_path, thresholded_color);

  std::vector<cv::Point2f> undistorted_thresholded_points = thresholded_points;
  cv::undistortImagePoints(undistorted_thresholded_points,
                           undistorted_thresholded_points, camera_matrix,
                           distortion_coefficients);
  cv::Mat undistorted_threshold_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  for (const cv::Point2f& point : undistorted_thresholded_points) {
    const cv::Point pixel(cvRound(point.x), cvRound(point.y));
    if (pixel.x >= 0 && pixel.x < undistorted_threshold_mask.cols &&
        pixel.y >= 0 && pixel.y < undistorted_threshold_mask.rows) {
      undistorted_threshold_mask.at<uint8_t>(pixel) = 255;
    }
  }
  const std::filesystem::path undistorted_threshold_output_path =
      output_folder_path /
      ("undistorted_threshold_" + image_path.stem().string() + ".png");
  WriteImage(undistorted_threshold_output_path, undistorted_threshold_mask);

  const auto lane_boundaries = tracker.GetImageLaneBoundaries(robot_pose);
  const auto clipped_lane_boundaries =
      ClipLaneBoundariesToImage(lane_boundaries, image.size());
  const auto densities = tracker.GetLaneDensities(image, robot_pose);
  const LaneRegions lane_regions =
      CalculateLaneRegions(clipped_lane_boundaries);
  cv::Mat annotated = AnnotateImage(undistorted_image, clipped_lane_boundaries,
                                    densities, lane_regions);
  const std::filesystem::path output_path =
      output_folder_path / image_path.filename();
  WriteImage(output_path, annotated);

  cv::Mat undistorted_threshold_overlay = undistorted_image.clone();
  cv::Mat threshold_color;
  cv::cvtColor(undistorted_threshold_mask, threshold_color, cv::COLOR_GRAY2BGR);
  cv::addWeighted(undistorted_threshold_overlay, 0.75, threshold_color, 0.25,
                  0.0, undistorted_threshold_overlay);
  const std::filesystem::path undistorted_overlay_output_path =
      output_folder_path /
      ("undistorted_threshold_overlay_" + image_path.stem().string() + ".png");
  WriteImage(undistorted_overlay_output_path, undistorted_threshold_overlay);

  cv::Mat annotated_field = AnnotateField(field_image, robot_pose);
  LOG(INFO) << "ROBOT POSE";
  utils::PrintPose3d(robot_pose);
  const std::filesystem::path field_output_path =
      output_folder_path / ("field_" + image_path.filename().string());
  WriteImage(field_output_path, annotated_field);
  LOG(INFO) << "Processed image: " << image_path << " -> " << output_path;

  return 0;
}
