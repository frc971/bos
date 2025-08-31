#include "pose_estimator.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>

namespace PoseEstimator {
using json = nlohmann::json;

frc971::apriltag::CameraMatrix camera_matrix_from_json(json intrinsics) {
  frc971::apriltag::CameraMatrix camMat = {.fx = intrinsics["fx"],
                                           .cx = intrinsics["cx"],
                                           .fy = intrinsics["fy"],
                                           .cy = intrinsics["cy"]};
  return camMat;
}

frc971::apriltag::DistCoeffs
distortion_coefficients_from_json(json intrinsics) {
  frc971::apriltag::DistCoeffs dist = {.k1 = intrinsics["k1"],
                                       .k2 = intrinsics["k2"],
                                       .p1 = intrinsics["p1"],
                                       .p2 = intrinsics["p2"],
                                       .k3 = intrinsics["k3"]};

  return dist;
}

PoseEstimator::PoseEstimator(frc971::apriltag::CameraMatrix camera_matrix,
                             frc971::apriltag::DistCoeffs dist_coeffs,
                             std::vector<cv::Point3f> apriltag_dimensions)
    : apriltag_layout_(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark)), camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs),
      apriltag_dimensions_(apriltag_dimensions) {

  apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 5;
  apriltag_detector_->debug = false;

  gpu_detector_ = new frc971::apriltag::GpuDetector(
      1456, 1088, apriltag_detector_, camera_matrix, dist_coeffs);
}
PoseEstimator::~PoseEstimator() {
  delete gpu_detector_;
  delete apriltag_detector_;
  return;
}

std::vector<position_estimate_t> PoseEstimator::Estimate(cv::Mat &input_img) {
  cv::Mat gray;
  cv::cvtColor(input_img, gray, cv::COLOR_BGR2GRAY);
  gpu_detector_->DetectGrayHost((unsigned char *)gray.ptr());
  const zarray_t *detections = gpu_detector_->Detections();
  std::vector<position_estimate_t> estimates;

  if (zarray_size(detections)) {
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t *gpu_detection;
      zarray_get(detections, i, &gpu_detection);
      std::printf("tag: %f, %f id: %d\n", gpu_detection->c[0],
                  gpu_detection->c[1], gpu_detection->id);
      cv::Point point(gpu_detection->c[0], gpu_detection->c[1]);

      std::vector<cv::Point2f> imagePoints;
      for (int i = 0; i < 4; ++i) {
        imagePoints.emplace_back(gpu_detection->p[i][0],
                                 gpu_detection->p[i][1]);
      }
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
      cv::Mat tvec =
          cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector
      cv::Mat cameraMatrix =
          (cv::Mat_<double>(3, 3) << camera_matrix_.fx, 0, camera_matrix_.cx, 0,
           camera_matrix_.fy, camera_matrix_.cy, 0, 0, 1);
      cv::Mat distCoeffs =
          cv::Mat::zeros(4, 1, CV_64FC1); // vector of distortion coefficients
      cv::solvePnP(apriltag_dimensions_, imagePoints, cameraMatrix, distCoeffs,
                   rvec, tvec);

      position_estimate_t estimate;
      estimate.translation.x = tvec.ptr<double>()[0];
      estimate.translation.y = tvec.ptr<double>()[1];
      estimate.translation.z = tvec.ptr<double>()[2];

      estimate.rotation.x = rvec.ptr<double>()[0];
      estimate.rotation.y = rvec.ptr<double>()[1];
      estimate.rotation.z = rvec.ptr<double>()[2];

      estimate.tag_id = gpu_detection->id;
      estimates.push_back(estimate);
      std::cout << "--- Pose Estimation Results ---" << "\n";
      std::cout << "Rotation Vector (rvec):\n" << rvec << "\n";
      std::cout << "Translation Vector (tvec):\n" << tvec << "\n";
      std::cout << "-------------------------------" << "\n";
      break;
    }
  }
  return estimates;
}


void PoseEstimator::TransformPose(position_estimate_t* estimate, int tag_id){
  frc::Pose3d tag_pose = apriltag_layout_.GetTagPose(tag_id).value();
}

} // namespace PoseEstimator
