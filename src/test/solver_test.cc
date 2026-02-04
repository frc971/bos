#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"

constexpr int kimage_tag_size = 50;
constexpr int ktag_id = 29;
constexpr int ktag_offset_y = 0;
constexpr int ktag_offset_x = 0;
constexpr camera::Camera kconfig = camera::Camera::DUMMY_CAMERA;

auto main() -> int {
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // output rotation vector
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // output translation vector
  auto camera_matrix =
      utils::camera_matrix_from_json<cv::Mat>(utils::read_intrinsics(
          camera::camera_constants[kconfig].intrinsics_path));

  auto distortion_coefficients =
      utils::distortion_coefficients_from_json<cv::Mat>(utils::read_intrinsics(
          camera::camera_constants[kconfig].intrinsics_path));

  localization::SquareSolver solver(kconfig);

  std::array<cv::Point2f, 4> image_points = {
      cv::Point2f(1000 - kimage_tag_size + ktag_offset_x,
                  500 + kimage_tag_size + ktag_offset_y),
      cv::Point2f(1000 + kimage_tag_size + ktag_offset_x,
                  500 + kimage_tag_size + ktag_offset_y),
      cv::Point2f(1000 + kimage_tag_size + ktag_offset_x,
                  500 - kimage_tag_size + ktag_offset_y),
      cv::Point2f(1000 - kimage_tag_size + ktag_offset_x,
                  500 - kimage_tag_size + ktag_offset_y),
  };
  localization::tag_detection_t dummy_tag_detection{
      .tag_id = ktag_id,
      .corners = image_points,
      .timestamp = 0.0,
      .confidence = 0.0,
  };

  localization::position_estimate_t position_estimate =
      solver.EstimatePosition({dummy_tag_detection})[0];

  utils::PrintPose3d(position_estimate.pose);
  LOG(INFO) << position_estimate;
}
