#pragma once

#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <sstream>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "position.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {
using json = nlohmann::json;

constexpr double ktag_size = 0.1651;  // meters
const std::vector<cv::Point3f> kapriltag_dimensions = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

typedef struct SpeedContraint {
  double translation_speed;
  double rotation_speed;
} speed_contraint_t;

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
      json intrinsics, json extrinsics, speed_contraint_t speed_contraint,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  ~TagEstimator();
  std::vector<tag_detection_t> Estimate(cv::Mat& frame) const;
  std::vector<tag_detection_t> GetRawPositionEstimates(cv::Mat& frame) const;

 private:
  // should be pointer?
  // Changes the position estimate to be tag relitive to absolute feild position
  tag_detection_t GetFeildRelitivePosition(
      tag_detection_t tag_relitive_position) const;
  tag_detection_t ApplyExtrinsics(tag_detection_t position) const;

 private:
  json extrinsics_;
  speed_contraint_t speed_contraint_;
  frc::AprilTagFieldLayout apriltag_layout_;
  apriltag_detector_t* apriltag_detector_;
  frc971::apriltag::GpuDetector* gpu_detector_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<cv::Point3f> apriltag_dimensions_;
  bool verbose_;
};
}  // namespace localization
