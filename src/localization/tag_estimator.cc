#include "tag_estimator.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <ntcore_cpp_types.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpilibc/frc/Timer.h>
#include <cmath>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include "src/localization/position.h"

namespace localization {

void PrintPose3d(const frc::Pose3d& pose) {
  // Extract translation (in meters)
  double x = pose.X().value();
  double y = pose.Y().value();
  double z = pose.Z().value();

  // Extract rotation (in degrees)
  double roll = pose.Rotation().X().value();  // radians → will convert below
  double pitch = pose.Rotation().Y().value();
  double yaw = pose.Rotation().Z().value();

  // Convert radians to degrees
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Pose3d -> X: " << x << " m, Y: " << y << " m, Z: " << z << " m"
            << ", Roll: " << roll << "°, Pitch: " << pitch << "°, Yaw: " << yaw
            << "°" << std::endl;
}

inline void PrintTransform3d(const frc::Transform3d& T) {
  const auto& tr = T.Translation();
  const auto& r = T.Rotation();

  fmt::print(
      "Transform3d: "
      "translation (x={:.3f} m, y={:.3f} m, z={:.3f} m), "
      "rotation (roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg)\n",
      tr.X().value(), tr.Y().value(), tr.Z().value(),
      units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(),
      units::degree_t{r.Z()}.value());
}

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

void PrintPositionEstimate(tag_detection_t estimate) {
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

void PrintPositionEstimates(std::vector<tag_detection_t> estimates) {
  for (tag_detection_t& estimate : estimates) {
    std::cout << "--- Pose Estimation Results ---"
              << "\n";
    PrintPositionEstimate(estimate);
  }
}

json ExtrinsicsToJson(tag_detection_t extrinsics) {
  json output;
  output["translation_x"] = extrinsics.translation.x;
  output["translation_y"] = extrinsics.translation.y;
  output["translation_z"] = extrinsics.translation.z;

  output["rotation_x"] = extrinsics.rotation.x;
  output["rotation_y"] = extrinsics.rotation.y;
  output["rotation_z"] = extrinsics.rotation.z;
  return output;
}

TagEstimator::TagEstimator(uint image_width, uint image_height, json intrinsics,
                           json extrinsics,
                           std::vector<cv::Point3f> apriltag_dimensions,
                           bool verbose)
    : extrinsics_(extrinsics),
      apriltag_layout_(frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeAndyMark)),
      camera_matrix_(camera_matrix_from_json<cv::Mat>(intrinsics)),
      distortion_coefficients_(
          distortion_coefficients_from_json<cv::Mat>(intrinsics)),
      apriltag_dimensions_(apriltag_dimensions),
      verbose_(true) {

  apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 4;
  apriltag_detector_->debug = false;

  gpu_detector_ = new frc971::apriltag::GpuDetector(
      image_width, image_height, apriltag_detector_,
      camera_matrix_from_json<frc971::apriltag::CameraMatrix>(intrinsics),
      distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
          intrinsics));
}

TagEstimator::~TagEstimator() {
  delete gpu_detector_;
  delete apriltag_detector_;
  return;
}

std::vector<tag_detection_t> TagEstimator::Estimate(cv::Mat& frame,
                                                    double timestamp) const {
  std::vector<tag_detection_t> estimates =
      GetRawPositionEstimates(frame, timestamp);
  for (tag_detection_t& estimate : estimates) {
    estimate = GetFeildRelitivePosition(estimate);
  }
  return estimates;
}

std::vector<tag_detection_t> TagEstimator::GetRawPositionEstimates(
    cv::Mat& frame, double timestamp) const {
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  gpu_detector_->DetectGrayHost((unsigned char*)gray.ptr());
  const zarray_t* detections = gpu_detector_->Detections();
  std::vector<tag_detection_t> estimates;

  if (zarray_size(detections)) {
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t* gpu_detection;
      zarray_get(detections, i, &gpu_detection);

      std::vector<cv::Point2f> imagePoints;
      for (int i = 0; i < 4; ++i) {

        imagePoints.emplace_back(gpu_detection->p[i][0],
                                 gpu_detection->p[i][1]);
      }
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
      cv::Mat tvec =
          cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
      cv::solvePnP(apriltag_dimensions_, imagePoints, camera_matrix_,
                   distortion_coefficients_, rvec, tvec, false,
                   cv::SOLVEPNP_IPPE_SQUARE);

      tag_detection_t estimate;
      // Currently we do not use transation z, rotation x and rotation y
      estimate.translation.x = tvec.ptr<double>()[2];
      estimate.translation.y = tvec.ptr<double>()[0];
      estimate.translation.z = tvec.ptr<double>()[1];

      estimate.rotation.x = rvec.ptr<double>()[2];
      estimate.rotation.y = rvec.ptr<double>()[0];
      estimate.rotation.z = rvec.ptr<double>()[1];

      estimate.timestamp = timestamp;

      estimate.distance =
          sqrt(square(estimate.translation.x) + square(estimate.translation.y));
      estimate.tag_id = gpu_detection->id;

      estimates.push_back(estimate);
    }
  }
  return estimates;
}

tag_detection_t TagEstimator::GetFeildRelitivePosition(
    const tag_detection_t tag_relative_position) const {

  frc::Transform3d camera_to_tag(
      units::meter_t{tag_relative_position.translation.x},
      units::meter_t{-tag_relative_position.translation.y},
      units::meter_t{-tag_relative_position.translation.z},
      frc::Rotation3d(
          units::radian_t{tag_relative_position.rotation.x},
          units::radian_t{-tag_relative_position.rotation.y},
          units::radian_t{-tag_relative_position.rotation.z} + 180_deg));

  frc::Transform3d tag_to_camera = camera_to_tag.Inverse();

  if (verbose_) {
    std::cout << "tag to camera: \n";
    PrintTransform3d(tag_to_camera);
    std::cout << "\n\n";
  }

  frc::Pose3d tag_pose =
      apriltag_layout_.GetTagPose(tag_relative_position.tag_id).value();

  if (verbose_) {
    std::cout << "tag id: " << tag_relative_position.tag_id << std::endl;
    std::cout << "tagpose: \n";
    PrintPose3d(tag_pose);
    std::cout << "\n\n";
  }

  frc::Pose3d camera_pose = tag_pose.TransformBy(tag_to_camera);

  if (verbose_) {
    std::cout << "camerapose: \n";
    PrintPose3d(camera_pose);
    std::cout << "\n\n";
  }

  frc::Transform3d robot_to_camera(
      units::meter_t{static_cast<double>(extrinsics_["translation_x"])},
      units::meter_t{static_cast<double>(extrinsics_["translation_y"])},
      units::meter_t{static_cast<double>(extrinsics_["translation_z"])},
      frc::Rotation3d(units::radian_t{extrinsics_["rotation_x"]},
                      units::radian_t{extrinsics_["rotation_y"]},
                      units::radian_t{extrinsics_["rotation_z"]}));
  frc::Transform3d camera_to_robot = robot_to_camera.Inverse();

  frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot);

  tag_detection_t field_relative_pose;

  field_relative_pose.tag_id = tag_relative_position.tag_id;

  field_relative_pose.rotation.x = robot_pose.Rotation().X().value();
  field_relative_pose.rotation.y = robot_pose.Rotation().Y().value();
  field_relative_pose.rotation.z = robot_pose.Rotation().Z().value();

  field_relative_pose.translation.x = robot_pose.Translation().X().value();
  field_relative_pose.translation.y = robot_pose.Translation().Y().value();
  field_relative_pose.translation.z = robot_pose.Translation().Z().value();

  field_relative_pose.distance = tag_relative_position.distance;

  field_relative_pose.timestamp = tag_relative_position.timestamp;

  return field_relative_pose;
}

tag_detection_t TagEstimator::ApplyExtrinsics(tag_detection_t position) const {
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

}  // namespace localization
