#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "third_party/971apriltag/971apriltag.h"

namespace PoseEstimator {
using json = nlohmann::json;

typedef struct Point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct PositionEstimate {
  point3d_t translation;
  point3d_t rotation;
  int tag_id;
} position_estimate_t;

constexpr double ktag_size = 0.1651;  // meters
const std::vector<cv::Point3f> kapriltag_dimensions = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

template <typename T>
T camera_matrix_from_json(json intrinsics);

template <typename T>
T distortion_coefficients_from_json(json intrinsics);

class PoseEstimator {
 public:
  PoseEstimator(
      json intrinsics, json extrinsics,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions);
  ~PoseEstimator();
  std::vector<position_estimate_t> Estimate(cv::Mat& input_img);

 private:
  // should be pointer?
  // Changes the position estimate to be tag relitive to absolute feild position
  position_estimate_t GetFeildRelitivePosition(
      position_estimate_t tag_relitive_position);
  position_estimate_t ApplyExtrinsics(position_estimate_t position);

 private:
  json extrinsics_;
  frc::AprilTagFieldLayout apriltag_layout_;
  apriltag_detector_t* apriltag_detector_;
  frc971::apriltag::GpuDetector* gpu_detector_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<cv::Point3f> apriltag_dimensions_;
};
}  // namespace PoseEstimator

#endif  // POSE_ESTIMATOR_H
