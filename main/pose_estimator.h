#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "971apriltag/971apriltag.h"
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

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
} position_estimate_t;

constexpr double ktag_size = 6.5; // TODO
const std::vector<cv::Point3f> kapriltag_dimensions = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

frc971::apriltag::CameraMatrix camera_matrix_from_json(json intrinsics);
frc971::apriltag::DistCoeffs distortion_coefficients_from_json(json intrinsics);

class PoseEstimator {
public:
  PoseEstimator(frc971::apriltag::CameraMatrix camera_matrix,
                frc971::apriltag::DistCoeffs dist_coeffs,
                std::vector<cv::Point3f> apriltag_dimensions);
  ~PoseEstimator();
  position_estimate_t Estimate(cv::Mat &input_img);

private:
  frc971::apriltag::GpuDetector *gpu_detector_;
  apriltag_detector_t *apriltag_detector_;
  frc971::apriltag::CameraMatrix camera_matrix_;
  frc971::apriltag::DistCoeffs dist_coeffs_;
  std::vector<cv::Point3f> apriltag_dimensions_;
};
} // namespace PoseEstimator

#endif // POSE_ESTIMATOR_H
