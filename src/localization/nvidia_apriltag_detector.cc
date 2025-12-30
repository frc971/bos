#include "src/localization/nvidia_apriltag_detector.h"
#include <vpi/Array.h>
#include <vpi/Stream.h>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include "src/localization/position.h"
#include "src/utils/log.h"

namespace localization {

frc::Pose3d Transform3dFromMatrix(float matrix[3][4]) {
  const float x_translation = matrix[2][3];
  const float y_translation = matrix[0][3];
  const float z_translation = matrix[1][3];

  Eigen::Matrix3d rotation_matrix{{matrix[2][2], matrix[2][0], matrix[2][1]},
                                  {matrix[0][2], matrix[0][0], matrix[0][1]},
                                  {matrix[1][2], matrix[1][0], matrix[1][1]}};
  Eigen::Quaterniond quaternion(rotation_matrix);
  return frc::Pose3d(
      frc::Translation3d(units::meter_t{x_translation},
                         units::meter_t{y_translation},
                         units::meter_t{z_translation}),
      frc::Rotation3d(frc::Quaternion(quaternion.w(), quaternion.x(),
                                      quaternion.y(), quaternion.z())));
}

NvidiaAprilTagDetector::NvidiaAprilTagDetector(
    int image_width, int image_height, nlohmann::json intrinsics,
    VPIAprilTagDecodeParams params, int max_detections,
    std::vector<cv::Point3f> apriltag_dimensions, bool verbose)
    : params_(params),
      max_detections_(max_detections),
      apriltag_dimensions_(apriltag_dimensions) {

  intrinsics_[0][0] = intrinsics["fx"];
  intrinsics_[0][2] = intrinsics["cx"];
  intrinsics_[1][1] = intrinsics["fy"];
  intrinsics_[1][2] = intrinsics["cy"];

  // According to nvidia benchmarks, VPI_BACKEND_CPU is better than pva
  CHECK(!vpiCreateAprilTagDetector(VPI_BACKEND_CPU, image_width, image_height,
                                   &params_, &payload_));

  CHECK(!vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION,
                        VPI_BACKEND_CPU, &detections_));
  CHECK(!vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU,
                        &poses_));
  CHECK(!vpiStreamCreate(0, &stream_));
}

std::vector<tag_detection_t> NvidiaAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame) {
  cv::Mat gray;

  if (timestamped_frame.frame.channels() == 1) {
    gray = timestamped_frame.frame;
  } else if (timestamped_frame.frame.channels() == 3) {
    cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
  }

  VPIImage input;

  CHECK(!vpiImageCreateWrapperOpenCVMat(gray, 0, &input));

  CHECK(!vpiSubmitAprilTagDetector(stream_, VPI_BACKEND_CPU, payload_,
                                   max_detections_, input, detections_));

  CHECK(!vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU, detections_,
                                         intrinsics_, ktag_size, poses_));

  CHECK(!vpiStreamSync(stream_));

  VPIArrayData detections_data;
  CHECK(!vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                          &detections_data));

  VPIArrayData poses_data;
  CHECK(!vpiArrayLockData(poses_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                          &poses_data));

  vpiImageDestroy(input);

  VPIPose* p = static_cast<VPIPose*>(poses_data.buffer.aos.data);
  if (*detections_data.buffer.aos.sizePointer != 0) {
    frc::Pose3d camera_relitive_position = Transform3dFromMatrix(p->transform);
    vpiArrayUnlock(detections_);
    vpiArrayUnlock(poses_);
    return std::vector<tag_detection_t>(
        {tag_detection_t{camera_relitive_position, timestamped_frame.timestamp,
                         std::hypot(camera_relitive_position.X().value(),
                                    camera_relitive_position.Y().value())}});
  }

  vpiArrayUnlock(detections_);
  vpiArrayUnlock(poses_);
  return std::vector<tag_detection_t>();
}

NvidiaAprilTagDetector::~NvidiaAprilTagDetector() {
  vpiStreamDestroy(stream_);
  vpiArrayDestroy(detections_);
  vpiArrayDestroy(poses_);
  vpiPayloadDestroy(payload_);
}
}  // namespace localization
