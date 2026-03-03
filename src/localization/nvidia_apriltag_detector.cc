#include "src/localization/nvidia_apriltag_detector.h"
#include <fmt/chrono.h>
#include <vpi/Array.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <Eigen/Geometry>
#include <vpi/OpenCVInterop.hpp>
#include "src/localization/position.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"
#include "src/utils/timer.h"

namespace localization {

auto Transform3dFromMatrix(float matrix[3][4]) -> frc::Pose3d {  // NOLINT
  // Need to convert to wpilib coordinates
  const float x_translation = matrix[2][3];
  const float y_translation = matrix[0][3];
  const float z_translation = matrix[1][3];

  Eigen::Matrix3d rotation_matrix{{matrix[2][2], matrix[2][0], matrix[2][1]},
                                  {matrix[0][2], matrix[0][0], matrix[0][1]},
                                  {matrix[1][2], matrix[1][0], matrix[1][1]}};

  // Converting to quaternion because wpilib fails to directly convert from matrix
  Eigen::Quaterniond quaternion(rotation_matrix);
  return {frc::Translation3d(units::meter_t{x_translation},
                             units::meter_t{y_translation},
                             units::meter_t{z_translation}),
          frc::Rotation3d(frc::Quaternion(quaternion.w(), quaternion.x(),
                                          quaternion.y(), quaternion.z()))};
}

NvidiaAprilTagDetector::NvidiaAprilTagDetector(int image_width,
                                               int image_height,
                                               nlohmann::json intrinsics,
                                               VPIAprilTagDecodeParams params,
                                               VPIBackend backend,
                                               int max_detections, bool verbose)
    : params_(params),
      backend_(backend),
      max_detections_(max_detections),
      input_(nullptr) {
  backend_ = VPI_BACKEND_CPU;

  CHECK(!vpiCreateAprilTagDetector(backend_, image_width, image_height,
                                   &params_, &payload_));

  CHECK(!vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION,
                        VPI_BACKEND_CPU, &detections_));
  CHECK(!vpiStreamCreate(0, &stream_));
}

auto NvidiaAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame)
    -> std::vector<tag_detection_t> {
  cv::Mat gray;

  if (timestamped_frame.frame.channels() == 1) {
    gray = timestamped_frame.frame;
  } else if (timestamped_frame.frame.channels() == 3) {
    cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
  }

  if (input_ == nullptr) {
    CHECK(!vpiImageCreateWrapperOpenCVMat(gray, VPI_BACKEND_CPU, &input_));
  } else {
    CHECK(!vpiImageSetWrappedOpenCVMat(input_, gray));
  }

  CHECK(!vpiSubmitAprilTagDetector(stream_, backend_, payload_, max_detections_,
                                   input_, detections_));

  CHECK(!vpiStreamSync(stream_));

  VPIArrayData detections_data{};
  CHECK(!vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                          &detections_data));

  std::vector<tag_detection_t> tag_detections;
  auto* detections =
      static_cast<VPIAprilTagDetection*>(detections_data.buffer.aos.data);
  int num_detections = *detections_data.buffer.aos.sizePointer;
  LOG(INFO) << num_detections;

  for (int i = 0; i < num_detections; ++i) {
    tag_detection_t detection;
    detection.tag_id = detections[i].id;
    detection.timestamp = timestamped_frame.timestamp;
    detection.confidence = 1;

    for (int j = 0; j < 4; ++j) {
      detection.corners[j] =
          cv::Point2f(detections[i].corners[j].x, detections[i].corners[j].y);
    }

    tag_detections.push_back(detection);
  }

  CHECK(!vpiArrayUnlock(detections_));
  return tag_detections;
}

NvidiaAprilTagDetector::~NvidiaAprilTagDetector() {
  vpiStreamDestroy(stream_);
  vpiArrayDestroy(detections_);
  vpiPayloadDestroy(payload_);
}
}  // namespace localization
