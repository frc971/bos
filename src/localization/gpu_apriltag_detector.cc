#include "src/localization/gpu_apriltag_detector.h"
#include <opencv2/calib3d.hpp>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "src/utils/constants_from_json.h"
#include "third_party/971apriltag/apriltag.h"

namespace localization {
using json = nlohmann::json;

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

GPUAprilTagDetector::GPUAprilTagDetector(uint image_width, uint image_height,
                                         const nlohmann::json& intrinsics,
                                         bool verbose)
    : camera_matrix_(utils::CameraMatrixFromJson<cv::Mat>(intrinsics)),
      distortion_coefficients_(
          utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics)) {

  LOG(INFO) << image_width << " " << image_height;

  apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 4;
  apriltag_detector_->debug = false;

  gpu_detector_ = std::make_unique<frc::apriltag::GpuDetector>(
      image_width, image_height, apriltag_detector_,
      utils::CameraMatrixFromJson<frc::apriltag::CameraMatrix>(intrinsics),
      utils::DistortionCoefficientsFromJson<frc::apriltag::DistCoeffs>(
          intrinsics),
      vision::ImageFormat::BGR8);
}
auto GPUAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame)
    -> std::vector<tag_detection_t> {
  try {
    if (timestamped_frame.frame.channels() == 1) {
      gpu_detector_->Detect((unsigned char*)timestamped_frame.frame.ptr(),
                            nullptr);
    } else if (timestamped_frame.frame.channels() == 3) {
      cv::Mat gray;
      cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
      gpu_detector_->Detect((unsigned char*)gray.ptr(), nullptr);
    } else {
      LOG(ERROR) << "Unknown frame type";
    }
  } catch (const std::exception& e) {
    LOG(WARNING) << "Caught exception: " << e.what();
    return {};
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
