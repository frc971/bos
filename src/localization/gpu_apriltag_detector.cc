#include "src/localization/gpu_apriltag_detector.h"
#include <frc/geometry/Transform3d.h>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {
using json = nlohmann::json;

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

template <>
auto camera_matrix_from_json<frc971::apriltag::CameraMatrix>(json intrinsics)
    -> frc971::apriltag::CameraMatrix {
  frc971::apriltag::CameraMatrix camera_matrix = {.fx = intrinsics["fx"],
                                                  .cx = intrinsics["cx"],
                                                  .fy = intrinsics["fy"],
                                                  .cy = intrinsics["cy"]};
  return camera_matrix;
}

template <>
auto distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
    json intrinsics) -> frc971::apriltag::DistCoeffs {
  frc971::apriltag::DistCoeffs distortion_coefficients = {
      .k1 = intrinsics["k1"],
      .k2 = intrinsics["k2"],
      .p1 = intrinsics["p1"],
      .p2 = intrinsics["p2"],
      .k3 = intrinsics["k3"]};

  return distortion_coefficients;
}

template <>
auto camera_matrix_from_json<cv::Mat>(json intrinsics) -> cv::Mat {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);
  return camera_matrix;
}

template <>
auto distortion_coefficients_from_json<cv::Mat>(json intrinsics) -> cv::Mat {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

GPUAprilTagDetector::GPUAprilTagDetector(
    uint image_width, uint image_height, const nlohmann::json& intrinsics,
    std::vector<cv::Point3f> apriltag_dimensions, bool verbose)
    : camera_matrix_(camera_matrix_from_json<cv::Mat>(intrinsics)),
      distortion_coefficients_(
          distortion_coefficients_from_json<cv::Mat>(intrinsics)),
      apriltag_dimensions_(std::move(apriltag_dimensions)),
      verbose_(verbose) {

  apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 4;
  apriltag_detector_->debug = false;

  gpu_detector_ = std::make_unique<frc971::apriltag::GpuDetector>(
      image_width, image_height, apriltag_detector_,
      camera_matrix_from_json<frc971::apriltag::CameraMatrix>(intrinsics),
      distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
          intrinsics));
}
auto GPUAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame)
    -> std::vector<tag_detection_t> {
  if (timestamped_frame.frame.channels() == 1) {
    gpu_detector_->DetectGrayHost(
        (unsigned char*)timestamped_frame.frame.ptr());
  } else if (timestamped_frame.frame.channels() == 3) {
    cv::Mat gray;
    cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
    gpu_detector_->DetectGrayHost((unsigned char*)gray.ptr());
  }
  const zarray_t* raw_detections = gpu_detector_->Detections();
  std::vector<tag_detection_t> tag_detections;

  if (zarray_size(raw_detections)) {
    for (int i = 0; i < zarray_size(raw_detections); ++i) {
      apriltag_detection_t* gpu_detection;
      zarray_get(raw_detections, i, &gpu_detection);

      tag_detection_t detection;
      detection.tag_id = gpu_detection->id;
      detection.timestamp = timestamped_frame.timestamp;
      detection.confidence = gpu_detection->decision_margin;

      for (int j = 0; j < 4; ++j) {
        detection.corners[j] =
            cv::Point2f(gpu_detection->p[j][0], gpu_detection->p[j][1]);
      }

      tag_detections.push_back(detection);
    }
  }
  return tag_detections;
}

GPUAprilTagDetector::~GPUAprilTagDetector() {
  if (apriltag_detector_ != nullptr) {
    apriltag_detector_destroy(apriltag_detector_);
  }
}

}  // namespace localization
