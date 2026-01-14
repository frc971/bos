#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <nlohmann/json.hpp>
#include "src/localization/apriltag_detector.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {

template <typename T>
auto camera_matrix_from_json(nlohmann::json intrinsics) -> T;

template <typename T>
auto distortion_coefficients_from_json(nlohmann::json intrinsics) -> T;

class GPUAprilTagDetector : public IAprilTagDetector {
 public:
  GPUAprilTagDetector(
      uint image_width, uint image_height, const nlohmann::json& intrinsics,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  ~GPUAprilTagDetector() override;
  auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<position_estimate_t> override;

 private:
  auto DetectTags(cv::Mat& frame) -> std::vector<std::vector<cv::Point2f>>;
  auto JointSolve(std::vector<std::vector<cv::Point2f>>& tag_detections)
      -> std::vector<position_estimate_t>;
  auto SquareSolve(std::vector<std::vector<cv::Point2f>>& tag_detections)
      -> std::vector<position_estimate_t>;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<cv::Point3f> apriltag_dimensions_;
  apriltag_detector_t* apriltag_detector_;
  std::unique_ptr<frc971::apriltag::GpuDetector> gpu_detector_;
  bool verbose_;
};
}  // namespace localization
