#include "src/gamepiece/lane_density.h"

#include <optional>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "src/gamepiece/gamepiece.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/image_utils.h"
#include "src/utils/transform.h"

namespace gamepiece {
namespace {

constexpr float kMinimumCameraDepth = 1e-3f;

static inline auto unhomogenize(const cv::Vec3f& v) -> cv::Vec2f {
  if (v[2] == 0) {
    return cv::Vec2f{v[0], v[1]};
  } else {
    return cv::Vec2f{v[0] / v[2], v[1] / v[2]};
  }
}

auto ClipToPositiveCameraDepth(cv::Vec4f& origin, cv::Vec4f& end) -> bool {
  if (origin[2] < kMinimumCameraDepth && end[2] < kMinimumCameraDepth) {
    return false;
  }

  if (origin[2] < kMinimumCameraDepth) {
    const float interpolation =
        (kMinimumCameraDepth - origin[2]) / (end[2] - origin[2]);
    origin += interpolation * (end - origin);
  } else if (end[2] < kMinimumCameraDepth) {
    const float interpolation =
        (kMinimumCameraDepth - end[2]) / (origin[2] - end[2]);
    end += interpolation * (origin - end);
  }
  return true;
}

}  // namespace

LaneDensityTracker::LaneDensityTracker(
    const camera::camera_constant_t& camera_constant) {
  if (!camera_constant.intrinsics_path.has_value()) {
    LOG(FATAL) << "Cannot run gamepiece without intrinsics";
  }
  if (!camera_constant.extrinsics_path.has_value()) {
    LOG(FATAL) << "Cannot run gamepiece without extrinsics";
  }
  const nlohmann::json intrinsics_json =
      utils::ReadIntrinsics(*camera_constant.intrinsics_path);
  camera_intrinsics_ =
      cv::Matx33f(utils::CameraMatrixFromJson<cv::Mat>(intrinsics_json));
  const nlohmann::json json_extrinsics =
      utils::ReadExtrinsics(*camera_constant.extrinsics_path);
  cv::Mat camera_extrinsics_cv = utils::EigenToCvMat(
      utils::ExtrinsicsJsonToCameraToRobot(json_extrinsics).ToMatrix());
  utils::ChangeBasis(camera_extrinsics_cv, utils::WPI_TO_CV);
  camera_extrinsics_cv.convertTo(camera_extrinsics_cv, CV_32F);
  camera_extrinsics_cv_ = cv::Matx44f(camera_extrinsics_cv);
  distortion_coeffs_ =
      utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics_json);
}

auto LaneDensityTracker::GetImageLaneBoundaries(const frc::Pose3d& robot_pose)
    -> std::array<image_lane_segment_t, 2 * num_lanes + 1> {
  cv::Mat robot_pose_cv = utils::EigenToCvMat(robot_pose.ToMatrix());
  utils::ChangeBasis(robot_pose_cv, utils::WPI_TO_CV);
  cv::Matx44f robot_pose_cv_mat(robot_pose_cv);
  const cv::Matx44f field_to_camera =
      (robot_pose_cv_mat * camera_extrinsics_cv_).inv();
  const cv::Matx34f camera_to_image = camera_intrinsics_ * Pi;

  std::array<image_lane_segment_t, 2 * num_lanes + 1> image_relative_lanes{};
  for (size_t i = 0; i < field_relative_lane_boundaries_.size(); ++i) {
    const lane_segment_t& lane = field_relative_lane_boundaries_[i];
    cv::Vec4f camera_relative_origin = field_to_camera * lane.origin;
    cv::Vec4f camera_relative_end = field_to_camera * lane.end;
    if (!ClipToPositiveCameraDepth(camera_relative_origin,
                                   camera_relative_end)) {
      continue;
    }
    const cv::Vec4f camera_relative_midpoint =
        (camera_relative_origin + camera_relative_end) * 0.5f;
    image_relative_lanes[i] = {
        .origin = unhomogenize(camera_to_image * camera_relative_origin),
        .end = unhomogenize(camera_to_image * camera_relative_end),
        .midpoint = unhomogenize(camera_to_image * camera_relative_midpoint),
        .valid = true,
    };
  }
  return image_relative_lanes;
}

auto LaneDensityTracker::GetLaneDensities(const cv::Mat& color_image,
                                          const frc::Pose3d& robot_pose)
    -> std::array<float, 2 * num_lanes> {
  std::vector<cv::Point2f> thresholded_points;
  cv::Mat3b hsv_image;
  cv::Mat1b threshold_mask;
  utils::HSVThreshold(color_image, hsv_color_range.first,
                      hsv_color_range.second, minimum_saturation,
                      thresholded_points, hsv_image, threshold_mask);
  cv::undistortImagePoints(thresholded_points, thresholded_points,
                           camera_intrinsics_, distortion_coeffs_);
  const auto image_relative_lanes = GetImageLaneBoundaries(robot_pose);
  std::array<float, 2 * num_lanes> per_lane_pixel_density{};
  std::optional<size_t> first_valid_lane;
  for (size_t i = 0; i + 1 < image_relative_lanes.size(); ++i) {
    if (image_relative_lanes[i].valid && image_relative_lanes[i + 1].valid) {
      first_valid_lane = i;
      break;
    }
  }
  if (!first_valid_lane.has_value()) {
    return per_lane_pixel_density;
  }

  const cv::Vec2f across_lanes =
      image_relative_lanes[*first_valid_lane + 1].midpoint -
      image_relative_lanes[*first_valid_lane].midpoint;
  const float across_lanes_norm = cv::norm(across_lanes);
  if (across_lanes_norm == std::numeric_limits<float>::epsilon()) {
    LOG(FATAL) << "Impossible: no distance between the lane midpoints";
  }
  const cv::Vec2f across_lanes_uvec = across_lanes / across_lanes_norm;
  const cv::Vec2f& signed_distance_origin =
      image_relative_lanes[*first_valid_lane].midpoint;
  std::array<float, 2 * num_lanes + 1> lane_boundary_distances{};
  for (size_t i = 0; i < image_relative_lanes.size(); ++i) {
    if (image_relative_lanes[i].valid) {
      lane_boundary_distances[i] =
          (image_relative_lanes[i].midpoint - signed_distance_origin)
              .dot(across_lanes_uvec);
    }
  }

  for (const cv::Point2f& image_point : thresholded_points) {
    const float point_distance =
        (static_cast<cv::Vec2f>(image_point) - signed_distance_origin)
            .dot(across_lanes_uvec);
    for (size_t lane_index = 0; lane_index + 1 < image_relative_lanes.size();
         ++lane_index) {
      if (!image_relative_lanes[lane_index].valid ||
          !image_relative_lanes[lane_index + 1].valid) {
        continue;
      }
      if (point_distance >= lane_boundary_distances[lane_index] &&
          point_distance < lane_boundary_distances[lane_index + 1]) {
        per_lane_pixel_density[lane_index] += 1.0f;
        break;
      }
    }
  }

  std::optional<std::pair<cv::Vec2f, cv::Vec2f>> prev_transformed_lane;
  for (size_t i = 0; i < image_relative_lanes.size(); i++) {
    if (!image_relative_lanes[i].valid) {
      if (i != 0) {
        per_lane_pixel_density[i - 1] = 0.0f;
      }
      prev_transformed_lane = std::nullopt;
      continue;
    }
    const auto& transformed_origin = image_relative_lanes[i].origin;
    const auto& transformed_end = image_relative_lanes[i].end;
    cv::Point clipped_origin{cvRound(transformed_origin[0]),
                             cvRound(transformed_origin[1])};
    cv::Point clipped_end{cvRound(transformed_end[0]),
                          cvRound(transformed_end[1])};
    const bool lane_is_visible =
        cv::clipLine(color_image.size(), clipped_origin, clipped_end);
    std::pair<cv::Vec2f, cv::Vec2f> curr_transformed_lane{
        cv::Vec2f{static_cast<float>(clipped_origin.x),
                  static_cast<float>(clipped_origin.y)},
        cv::Vec2f{static_cast<float>(clipped_end.x),
                  static_cast<float>(clipped_end.y)}};
    if (i != 0) {
      if (!lane_is_visible || !prev_transformed_lane.has_value()) {
        per_lane_pixel_density[i - 1] = 0.0f;
        prev_transformed_lane = lane_is_visible
                                    ? std::make_optional(curr_transformed_lane)
                                    : std::nullopt;
        continue;
      }

      cv::Vec2f diagonal =
          curr_transformed_lane.second - prev_transformed_lane->first;
      float diag_len = cv::norm(diagonal);
      if (diag_len == 0.0f) {
        per_lane_pixel_density[i - 1] = 0.0f;
        prev_transformed_lane = std::move(curr_transformed_lane);
        continue;
      }
      diagonal /= diag_len;
      cv::Vec2f offset_1 =
          curr_transformed_lane.first - prev_transformed_lane->first;
      cv::Vec2f perpendicular_component_1 =
          offset_1 - diagonal.dot(offset_1) * diagonal;
      cv::Vec2f offset_2 =
          prev_transformed_lane->second - prev_transformed_lane->first;
      cv::Vec2f perpendicular_component_2 =
          offset_2 - diagonal.dot(offset_2) * diagonal;
      float quadrilateral_area = 0.5 * diag_len *
                                 (cv::norm(perpendicular_component_1) +
                                  cv::norm(perpendicular_component_2));
      if (quadrilateral_area > 0.0f) {
        per_lane_pixel_density[i - 1] /= quadrilateral_area;
      } else {
        per_lane_pixel_density[i - 1] = 0.0f;
      }
    }
    prev_transformed_lane = lane_is_visible
                                ? std::make_optional(curr_transformed_lane)
                                : std::nullopt;
  }
  return per_lane_pixel_density;
}
}  // namespace gamepiece
