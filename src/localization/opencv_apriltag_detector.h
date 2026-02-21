#pragma once
#include <opencv2/objdetect/aruco_detector.hpp>
#include "src/localization/apriltag_detector.h"
#include "src/utils/pch.h"

namespace localization {

// AprilTag detector backed by OpenCV's built-in ArUco/AprilTag detection.
// Uses the DICT_APRILTAG_36h11 dictionary to match the tag36h11 family used
// across the rest of this codebase.
//
// Compared to GPUAprilTagDetector this runs entirely on the CPU, which makes
// it a useful fallback when CUDA is unavailable (e.g. desktop development
// machines) and a convenient reference implementation for testing.
class OpenCVAprilTagDetector : public IAprilTagDetector {
 public:
  // intrinsics: JSON object with keys fx, fy, cx, cy, k1, k2, k3, p1, p2.
  // Undistortion is applied to the detected corners before they are returned so
  // that downstream solvers receive undistorted image coordinates, consistent
  // with the behaviour of GPUAprilTagDetector.
  explicit OpenCVAprilTagDetector(const nlohmann::json& intrinsics);

  auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<tag_detection_t> override;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  cv::aruco::ArucoDetector detector_;
};

}  // namespace localization
