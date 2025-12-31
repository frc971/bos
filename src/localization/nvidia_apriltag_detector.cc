#include "src/localization/nvidia_apriltag_detector.h"
#include <fmt/chrono.h>
#include <vpi/Array.h>
#include <vpi/Stream.h>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include "src/localization/position.h"
#include "src/utils/log.h"
#include "src/utils/timer.h"

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
    VPIAprilTagDecodeParams params, VPIBackend backend, int max_detections,
    std::vector<cv::Point3f> apriltag_dimensions, bool verbose)
    : params_(params),
      backend_(backend),
      max_detections_(max_detections),
      apriltag_dimensions_(apriltag_dimensions),
      input_(nullptr) {

  intrinsics_[0][0] = intrinsics["fx"];
  intrinsics_[0][2] = intrinsics["cx"];
  intrinsics_[1][1] = intrinsics["fy"];
  intrinsics_[1][2] = intrinsics["cy"];

  (vpiCreateAprilTagDetector(backend_, image_width, image_height, &params_,
                             &payload_));

  (vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION, 0,
                  &detections_));
  (vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_POSE, 0, &poses_));
  (vpiStreamCreate(0, &stream_));
}

std::vector<tag_detection_t> NvidiaAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame) {
  cv::Mat gray;

  if (timestamped_frame.frame.channels() == 1) {
    gray = timestamped_frame.frame;
  } else if (timestamped_frame.frame.channels() == 3) {
    cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
  }

  if (input_ == nullptr) {
    std::cout << "input is nullptr" << std::endl;
    std::cout << gray.size << std::endl;
    std::cout << gray.channels() << std::endl;
    std::cout << (vpiImageCreateWrapperOpenCVMat(gray, 0, &input_))
              << std::endl;
  } else {
    std::cout << "input is not nullptr" << std::endl;
    std::cout << gray.size << std::endl;
    std::cout << gray.channels() << std::endl;
    std::cout << (vpiImageSetWrappedOpenCVMat(input_, gray)) << std::endl;
  }

  (vpiSubmitAprilTagDetector(stream_, backend_, payload_, max_detections_,
                             input_, detections_));

  (vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU, detections_,
                                   intrinsics_, ktag_size, poses_));

  vpiStreamSync(stream_);

  VPIArrayData detections_data{};
  vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                   &detections_data);

  VPIArrayData poses_data{};
  vpiArrayLockData(poses_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                   &poses_data);

  VPIPose* p = static_cast<VPIPose*>(poses_data.buffer.aos.data);
  if (*detections_data.buffer.aos.sizePointer != 0) {
    frc::Pose3d camera_relitive_position = Transform3dFromMatrix(p->transform);
    PrintPose3d(camera_relitive_position);
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
