#pragma once
#include <array>
#include "src/camera/camera_constants.h"

namespace gamepiece {

class LaneDensityTracker {
 public:
  LaneDensityTracker(const camera::camera_constant_t& camera);
  auto GetLaneDensities(const cv::Mat& rgb_image, const frc::Pose3d& robot_pose)
      -> std::vector<double>;

 private:
  cv::Matx44f camera_extrinsics_cv_;
  cv::Matx33f camera_intrinsics_;
  cv::Vec<double, 5> distortion_coeffs_;
  // meters, wpilib coordinates
  static constexpr float lane_width = 1.0;
  static constexpr int num_lanes =
      2;  // actually 2x because this is reflected across center line
  static constexpr float center_field_x = 8.256524;
  static constexpr float field_width = 8.07;
  static constexpr float lane_begin_y = 1.0;
  inline static const cv::Matx34f Pi = cv::Matx34f(  // clang-format off
         1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0);  // clang-format on
  inline static const cv::Vec4f field_relative_lane_direction{
      -(field_width - lane_begin_y * 2), 0, 0, 0};
  inline static const cv::Vec4f field_relative_interlane_offset{0, 0,
                                                                lane_width, 0};
};
}  // namespace gamepiece
