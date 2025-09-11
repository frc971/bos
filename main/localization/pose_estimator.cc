#include "pose_estimator.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>
#include <ntcore_cpp_types.h>
#include <cmath>
#include <fstream>

#define PRINT_DETECTION_POSE false

namespace PoseEstimator {
using json = nlohmann::json;

constexpr double square(double x) {
  return x * x;
}

constexpr double RadianToDegree(double radian) {
  return radian * (180 / M_PI);
}

template <>
frc971::apriltag::CameraMatrix
camera_matrix_from_json<frc971::apriltag::CameraMatrix>(json intrinsics) {
  frc971::apriltag::CameraMatrix camera_matrix = {.fx = intrinsics["fx"],
                                                  .cx = intrinsics["cx"],
                                                  .fy = intrinsics["fy"],
                                                  .cy = intrinsics["cy"]};
  return camera_matrix;
}

template <>
frc971::apriltag::DistCoeffs
distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
    json intrinsics) {
  frc971::apriltag::DistCoeffs distortion_coefficients = {
      .k1 = intrinsics["k1"],
      .k2 = intrinsics["k2"],
      .p1 = intrinsics["p1"],
      .p2 = intrinsics["p2"],
      .k3 = intrinsics["k3"]};

  return distortion_coefficients;
}

template <>
cv::Mat camera_matrix_from_json<cv::Mat>(json intrinsics) {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);
  return camera_matrix;
}

template <>
cv::Mat distortion_coefficients_from_json<cv::Mat>(json intrinsics) {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

PoseEstimator::PoseEstimator(json intrinsics, json extrinsics,
                             std::vector<cv::Point3f> apriltag_dimensions)
    : extrinsics_(extrinsics),
      apriltag_layout_(frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeAndyMark)),
      camera_matrix_(camera_matrix_from_json<cv::Mat>(intrinsics)),
      distortion_coefficients_(
          distortion_coefficients_from_json<cv::Mat>(intrinsics)),
      apriltag_dimensions_(apriltag_dimensions) {

  apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 4;
  apriltag_detector_->debug = false;
  apriltag_detector_->quad_decimate = 1;

  gpu_detector_ = new frc971::apriltag::GpuDetector(
      1456, 1088, apriltag_detector_,
      camera_matrix_from_json<frc971::apriltag::CameraMatrix>(intrinsics),
      distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
          intrinsics),
      1);
}
PoseEstimator::~PoseEstimator() {
  delete gpu_detector_;
  delete apriltag_detector_;
  return;
}

std::vector<position_estimate_t> PoseEstimator::Estimate(cv::Mat& frame) {
  std::vector<position_estimate_t> estimates = GetRawPositionEstimates(frame);
  for (position_estimate_t& estimate : estimates) {
    estimate = GetFeildRelitivePosition(estimate);
    PrintPositionEstimates(estimates);
    // estimate = ApplyExtrinsics(estimate);
  }
  if (PRINT_DETECTION_POSE) {
    PrintPositionEstimates(estimates);
  }
  return estimates;
}

void PoseEstimator::PrintPositionEstimates(
    std::vector<position_estimate_t> estimates) {
  for (position_estimate_t& estimate : estimates) {
    std::cout << "--- Pose Estimation Results ---"
              << "\n";
    std::cout << "id: " << estimate.tag_id << "\n";
    std::cout << "Translation: "
              << "\n";
    std::cout << estimate.translation.x << "\n";
    std::cout << estimate.translation.y << "\n";
    std::cout << estimate.translation.z << "\n";
    std::cout << "Rotation: "
              << "\n";
    std::cout << RadianToDegree(estimate.rotation.x) << "\n";
    std::cout << RadianToDegree(estimate.rotation.y) << "\n";
    std::cout << RadianToDegree(estimate.rotation.z) << "\n";
  }
}
std::vector<position_estimate_t> PoseEstimator::GetRawPositionEstimates(
    cv::Mat& frame) {
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  gpu_detector_->DetectGrayHost((unsigned char*)gray.ptr());
  const zarray_t* detections = gpu_detector_->Detections();
  std::vector<position_estimate_t> estimates;

  if (zarray_size(detections)) {
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t* gpu_detection;
      zarray_get(detections, i, &gpu_detection);
      cv::Point point(gpu_detection->c[0], gpu_detection->c[1]);

      std::vector<cv::Point2f> imagePoints;
      for (int i = 0; i < 4; ++i) {
        imagePoints.emplace_back(gpu_detection->p[i][0],
                                 gpu_detection->p[i][1]);
      }
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
      cv::Mat tvec =
          cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
      cv::solvePnP(apriltag_dimensions_, imagePoints, camera_matrix_,
                   distortion_coefficients_, rvec, tvec);

      position_estimate_t estimate;
      // Currently we do not use transation z, rotation x and rotation y
      estimate.translation.x = tvec.ptr<double>()[2];
      estimate.translation.y = tvec.ptr<double>()[0];
      estimate.translation.z = tvec.ptr<double>()[1];

      estimate.rotation.x = rvec.ptr<double>()[2];
      estimate.rotation.y = rvec.ptr<double>()[0];
      estimate.rotation.z = rvec.ptr<double>()[1];

      estimate.tag_id = gpu_detection->id;

      estimates.push_back(estimate);
    }
  }
  return estimates;
}

position_estimate_t PoseEstimator::GetFeildRelitivePosition(
    position_estimate_t tag_relitive_position) {
  std::cout << "April tag rotation: "
            << apriltag_layout_.GetTagPose(tag_relitive_position.tag_id)
                   ->Rotation()
                   .Z()
                   .value()
            << "\n";
  position_estimate_t feild_relitive_position;
  feild_relitive_position.tag_id = tag_relitive_position.tag_id;

  feild_relitive_position.rotation.x = tag_relitive_position.rotation.x;
  feild_relitive_position.rotation.y = tag_relitive_position.rotation.y;
  feild_relitive_position.rotation.z = tag_relitive_position.rotation.z;

  feild_relitive_position.translation.x =
      sin(feild_relitive_position.rotation.z) *
          tag_relitive_position.translation.y +
      cos(feild_relitive_position.rotation.z) *
          tag_relitive_position.translation.x;
  feild_relitive_position.translation.y =
      cos(feild_relitive_position.rotation.z) *
          tag_relitive_position.translation.y -
      sin(feild_relitive_position.rotation.z) *
          tag_relitive_position.translation.x;
  feild_relitive_position.translation.z = tag_relitive_position.translation.z;

  double angle =
      -(M_PI / 2 - std::atan2(feild_relitive_position.translation.x,
                              feild_relitive_position.translation.y)) +
      apriltag_layout_.GetTagPose(tag_relitive_position.tag_id)
          ->Rotation()
          .Z()
          .value();

  double magnitude = sqrt(feild_relitive_position.translation.x *
                              feild_relitive_position.translation.x +
                          feild_relitive_position.translation.y *
                              feild_relitive_position.translation.y);

  feild_relitive_position.translation.x = std::cos(angle) * magnitude;
  feild_relitive_position.translation.y = std::sin(angle) * magnitude;

  feild_relitive_position.translation.x +=
      apriltag_layout_.GetTagPose(feild_relitive_position.tag_id)->X().value();
  feild_relitive_position.translation.y +=
      apriltag_layout_.GetTagPose(feild_relitive_position.tag_id)->Y().value();
  feild_relitive_position.translation.z +=
      apriltag_layout_.GetTagPose(feild_relitive_position.tag_id)->Z().value();

  return feild_relitive_position;
}

position_estimate_t PoseEstimator::ApplyExtrinsics(
    position_estimate_t position) {
  if (extrinsics_ == nullptr) {
    return position;
  }
  position.translation.x += static_cast<double>(extrinsics_["translation_x"]);
  position.translation.y += static_cast<double>(extrinsics_["translation_y"]);
  position.translation.z += static_cast<double>(extrinsics_["translation_z"]);

  position.rotation.x += static_cast<double>(extrinsics_["rotation_x"]);
  position.rotation.y += static_cast<double>(extrinsics_["rotation_y"]);
  position.rotation.z += static_cast<double>(extrinsics_["rotation_z"]);

  return position;
}

}  // namespace PoseEstimator
