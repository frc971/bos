#include "src/localization/nvidia_apriltag_detector.h"
#include <vpi/Array.h>
#include <vpi/Stream.h>
#include <Eigen/Geometry>
#include <vpi/OpenCVInterop.hpp>
#include "src/localization/position.h"
#include "src/utils/log.h"

namespace localization {

const int kmax_detections = 16;

frc::Transform3d Transform3dFromMatrix(float matrix[3][4]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      std::cout << matrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
  // Transform coordinates to wpi
  const float x_translation = matrix[2][3];
  const float y_translation = matrix[0][3];
  const float z_translation = matrix[1][3];

  Eigen::Matrix3d rotation_matrix{{matrix[2][2], matrix[2][0], matrix[2][1]},
                                  {matrix[0][2], matrix[0][0], matrix[0][1]},
                                  {matrix[1][2], matrix[1][0], matrix[1][1]}};
  Eigen::Quaterniond quaternion(rotation_matrix);
  return frc::Transform3d(
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

  vpiCreateAprilTagDetector(VPI_BACKEND_CPU, image_width, image_height, &params,
                            &payload_);

  vpiArrayCreate(max_detections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION,
                 VPI_BACKEND_CPU, &detections_);
  vpiArrayCreate(kmax_detections, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU,
                 &poses_);
  vpiStreamCreate(0, &stream_);
}

std::vector<tag_detection_t> NvidiaAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& frame) {

  VPIImage input;
  std::vector<tag_detection_t> a;
  vpiImageCreateWrapperOpenCVMat(frame.frame, 0, &input);

  vpiSubmitAprilTagDetector(stream_, VPI_BACKEND_CPU, payload_, 64, input,
                            detections_);

  vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU, detections_,
                                  intrinsics_, ktag_size, poses_);

  vpiStreamSync(stream_);

  VPIArrayData detections_data;
  vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                   &detections_data);

  VPIArrayData poses_data;
  vpiArrayLockData(poses_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                   &poses_data);

  VPIPose* p = (VPIPose*)poses_data.buffer.aos.data;
  if ((*detections_data.buffer.aos.sizePointer != 0)) {
    std::cout << "Got detections"
              << "\n";
    frc::Transform3d camera_relitive_position =
        Transform3dFromMatrix(p->transform);
    PrintTransform3d(camera_relitive_position);
  }

  vpiArrayUnlock(detections_);
  vpiArrayUnlock(poses_);
  return a;
}
}  // namespace localization
