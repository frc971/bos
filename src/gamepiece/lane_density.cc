#include "src/gamepiece/lane_density.h"
#include "src/gamepiece/gamepiece.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/image_utils.h"
#include "src/utils/transform.h"

namespace gamepiece {
auto signum(float val) -> int {
  return (0 < val) - (val < 0);
}

static inline auto unhomogenize(const cv::Vec3f& v) -> cv::Vec2f {
  if (v[2] == 0) {
    return cv::Vec2f{v[0], v[1]};
  } else {
    return cv::Vec2f{v[0] / v[2], v[1] / v[2]};
  }
}

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
  cv::Mat _robot_pose = utils::EigenToCvMat(robot_pose.ToMatrix());
  utils::ChangeBasis(_robot_pose, utils::WPI_TO_CV);
  cv::Matx44f robot_pose_cv(_robot_pose);
  const cv::Matx34f composed_pnp_mat_ =
      camera_intrinsics_ * Pi * (robot_pose_cv * camera_extrinsics_cv_).inv();
  const cv::Vec2f image_relative_lane_direction =
      unhomogenize(composed_pnp_mat_ * field_relative_lane_direction);
  const cv::Vec2f image_relative_lane_direction_uvec =
      image_relative_lane_direction / cv::norm(image_relative_lane_direction);
  const cv::Vec2f image_relative_center_line_origin = unhomogenize(
      composed_pnp_mat_ * cv::Vec4f{-lane_begin_y, 0, center_field_x, 1});
  std::vector<cv::Vec3f> image_relative_lane_origins;
  std::array<float, 2 * num_lanes> per_lane_pixel_density{};
  for (const cv::Point2f& image_point : thresholded_points) {
    auto offset =
        static_cast<cv::Vec2f>(image_point) - image_relative_center_line_origin;
    if (std::abs(std::acos(offset.dot(image_relative_lane_direction) /
                           cv::norm(offset))) > std::numbers::pi) {
      continue;
    }
    const cv::Vec2f parallel_offset =
        (image_relative_lane_direction_uvec.dot(offset)) *
        image_relative_lane_direction_uvec;
    if (cv::norm(parallel_offset) > field_width - lane_begin_y * 2) {
      continue;
    }
    const cv::Vec2f perpendicular_offset =
        static_cast<cv::Vec2f>(image_point) - parallel_offset;
    int lane_widths = cv::norm(perpendicular_offset) / lane_width;
    if (std::abs(lane_widths) > num_lanes) {
      continue;
    }
    int direction_flipper =
        signum(image_relative_lane_direction_uvec[0] * perpendicular_offset[1] -
               image_relative_lane_direction_uvec[1] * perpendicular_offset[0]);
    lane_widths *= direction_flipper;
    lane_widths = std::floor(lane_widths);
    per_lane_pixel_density[static_cast<int>(lane_widths) + num_lanes] += 1;
  }
  std::pair<cv::Vec2f, cv::Vec2f> prev_transformed_lane;
  for (size_t i = 0; i < field_relative_lanes.size(); i++) {
    std::pair<cv::Vec2f, cv::Vec2f> curr_transformed_lane{
        unhomogenize(composed_pnp_mat_ * field_relative_lanes[i].origin),
        unhomogenize(composed_pnp_mat_ * field_relative_lanes[i].end)};
    if (curr_transformed_lane[0] > color_image.cols) {}
    if (i != 0) {
      cv::Vec2f diagonal =
          curr_transformed_lane.second - prev_transformed_lane.first;
      float diag_len = cv::norm(diagonal);
      diagonal /= diag_len;
      cv::Vec2f offset_1 =
          curr_transformed_lane.first - prev_transformed_lane.first;
      cv::Vec2f perpendicular_component_1 =
          offset_1 - diagonal.dot(offset_1) * diagonal;
      cv::Vec2f offset_2 =
          prev_transformed_lane.second - prev_transformed_lane.first;
      cv::Vec2f perpendicular_component_2 =
          offset_2 - diagonal.dot(offset_2) * diagonal;
      float quadrilateral_area = 0.5 * diag_len *
                                 (cv::norm(perpendicular_component_1) +
                                  cv::norm(perpendicular_component_2));
      per_lane_pixel_density[i - 1] /= quadrilateral_area;
    }
    prev_transformed_lane = std::move(curr_transformed_lane);
  }
  return per_lane_pixel_density;
}
}  // namespace gamepiece
