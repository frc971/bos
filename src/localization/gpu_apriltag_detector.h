#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <nlohmann/json.hpp>
#include "src/localization/apriltag_detector.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {

template <typename T>
T camera_matrix_from_json(nlohmann::json intrinsics);

template <typename T>
T distortion_coefficients_from_json(nlohmann::json intrinsics);

class GPUAprilTagDetector : public IAprilTagDetector {
 public:
  GPUAprilTagDetector(
      uint image_width, uint image_height, nlohmann::json intrinsics,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  ~GPUAprilTagDetector();
  std::vector<tag_detection_t> GetTagDetections(
      camera::timestamped_frame_t& frame) override;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<cv::Point3f> apriltag_dimensions_;
  apriltag_detector_t* apriltag_detector_;
  std::unique_ptr<frc971::apriltag::GpuDetector> gpu_detector_;
  bool verbose_;
};
}  // namespace localization
