#include "src/gamepiece/lane_density.h"
#include "src/gamepiece/gamepiece.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace gamepiece {
auto signum(float val) -> int {
  return (0 < val) - (val < 0);
}

auto roundAwayFromZero(float num) -> float {
  if (num > 0.0) {
    return std::ceil(num);
  } else {
    return std::floor(num);
  }
}

static inline auto unhomogenize(const cv::Vec3f& v) -> cv::Vec2f {
  return cv::Vec2f{v[0], v[1]};
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
    -> std::vector<double> {
  std::vector<cv::Point2f> thresholded_points;
  cv::Mat3b hsv_image;
  cv::Mat1b threshold_mask;
  utils::HSVThreshold(color_image, hsv_color_range.first,
                      hsv_color_range.second, minimum_saturation,
                      thresholded_points, hsv_image, threshold_mask,
                      camera_intrinsics_);
  cv::undistortImagePoints(thresholded_points, thresholded_points,
                           camera_intrinsics_, distortion_coeffs_);
  cv::Mat _robot_pose = utils::EigenToCvMat(robot_pose.ToMatrix());
  utils::ChangeBasis(_robot_pose, utils::WPI_TO_CV);
  cv::Matx44f robot_pose_cv(_robot_pose);
  const cv::Matx34f composed_pnp_mat_ =
      camera_intrinsics_ * Pi * robot_pose_cv * camera_extrinsics_cv_;
  const cv::Vec2f image_relative_lane_direction =
      unhomogenize(composed_pnp_mat_ * field_relative_lane_direction);
  const cv::Vec2f image_relative_lane_direction_uvec =
      image_relative_lane_direction / cv::norm(image_relative_lane_direction);
  const cv::Vec2f image_relative_center_line_origin = unhomogenize(
      composed_pnp_mat_ * cv::Vec4f{-lane_begin_y, 0, center_field_x, 1});
  std::vector<cv::Vec3f> image_relative_lane_origins;
  std::vector<int> pixels_per_lane{num_lanes * 2};
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
        static_cast<cv::Vec2f>(image_point) - offset;
    int lane_widths = cv::norm(perpendicular_offset) / lane_width;
    if (lane_widths > num_lanes) {
      continue;
    }
    int direction_flipper =
        signum(image_relative_lane_direction_uvec[0] * perpendicular_offset[1] -
               image_relative_lane_direction_uvec[1] * perpendicular_offset[0]);
    lane_widths *= direction_flipper;
    lane_widths = roundAwayFromZero(lane_widths);
    pixels_per_lane[lane_widths] += 1;
  }
}
}  // namespace gamepiece
