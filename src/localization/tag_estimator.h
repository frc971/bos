#pragma once
#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <sstream>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "position.h"
#include "src/localization/apriltag_detector.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {
using json = nlohmann::json;

template <typename T>
T camera_matrix_from_json(json intrinsics);

template <typename T>
T distortion_coefficients_from_json(json intrinsics);

json ExtrinsicsToJson(tag_detection_t extrinsics);

// Estimates position based on each tag detection
// The position returned in feild reltive, wpilib coordinates
class TagEstimator {
 public:
  TagEstimator(
      uint image_width, uint image_height, json intrinsics, json extrinsics,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  TagEstimator(
      uint image_width, uint image_height, std::string intrinsics_path,
      std::string extrinsics_path,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  ~TagEstimator();
  std::vector<tag_detection_t> Estimate(cv::Mat& frame, double timestamp) const;
  std::vector<tag_detection_t> GetRawPositionEstimates(cv::Mat& frame,
                                                       double timestamp) const;
  tag_detection_t GetFeildRelitivePosition(
      tag_detection_t tag_relative_position) const;

 private:
  // should be pointer?
  // Changes the position estimate to be tag relative to absolute feild position
  tag_detection_t ApplyExtrinsics(tag_detection_t position) const;

 private:
  json extrinsics_;
  frc::AprilTagFieldLayout apriltag_layout_;
  apriltag_detector_t* apriltag_detector_;
  std::unique_ptr<frc971::apriltag::GpuDetector> gpu_detector_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<cv::Point3f> apriltag_dimensions_;
  bool verbose_;
};
}  // namespace localization
