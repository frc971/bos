#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <sys/types.h>
#include <vpi/Array.h>
#include <vpi/ArrayType.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/Types.h>
#include <vpi/algo/AprilTags.h>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"

frc::Transform3d Transform3dFromMatrix(float matrix[3][4]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      std::cout << matrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
  const float x_translation = matrix[2][3];
  const float y_translation = matrix[0][3];
  const float z_translation = matrix[1][3];

  Eigen::Matrix3d rotation_matrix{{matrix[2][2], matrix[2][0], matrix[2][1]},
                                  {matrix[0][1], matrix[0][0], matrix[0][1]},
                                  {matrix[1][2], matrix[1][0], matrix[1][1]}};
  Eigen::Quaterniond quaternion(rotation_matrix);
  return frc::Transform3d(
      frc::Translation3d(units::meter_t{x_translation},
                         units::meter_t{y_translation},
                         units::meter_t{z_translation}),
      frc::Rotation3d(frc::Quaternion(quaternion.w(), quaternion.x(),
                                      quaternion.y(), quaternion.z())));
}

int main() {
  // camera::Camera config = camera::SelectCameraConfig();
  camera::Camera config = camera::Camera::DEFAULT_USB0;
  camera::CameraSource source("nvidia_apriltag_test",
                              camera::GetCameraStream(config));
  cv::Mat mat = source.GetFrame();

  camera::CscoreStreamer streamer(source.GetName(), 4971, 30, 1080, 1080);

  nlohmann::json intrinsics_json =
      utils::read_intrinsics(camera::camera_constants[config].intrinsics_path);
  VPICameraIntrinsic intrinsics{
      {intrinsics_json["fx"], 0.0f, intrinsics_json["cx"]},
      {0.0f, intrinsics_json["fy"], intrinsics_json["cy"]}};

  VPIImage input;
  vpiImageCreateWrapperOpenCVMat(mat, 0, &input);

  int32_t w, h;
  vpiImageGetSize(input, &w, &h);

  VPIPayload payload;
  const int maxHamming = 2;
  const VPIAprilTagFamily family = VPI_APRILTAG_36H11;
  VPIAprilTagDecodeParams params = {NULL, 0, maxHamming, family};
  vpiCreateAprilTagDetector(VPI_BACKEND_CPU, w, h, &params, &payload);

  const int maxDetections = 64;

  VPIArray detections;
  vpiArrayCreate(maxDetections, VPI_ARRAY_TYPE_APRILTAG_DETECTION,
                 VPI_BACKEND_CPU, &detections);
  VPIArray poses;
  vpiArrayCreate(maxDetections, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU, &poses);

  VPIStream stream;
  vpiStreamCreate(0, &stream);

  const float tagSize = 0.2f;
  while (true) {
    mat = source.GetFrame();
    cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);
    streamer.WriteFrame(mat);

    vpiImageCreateWrapperOpenCVMat(mat, 0, &input);

    vpiSubmitAprilTagDetector(stream, VPI_BACKEND_CPU, payload, maxDetections,
                              input, detections);

    vpiSubmitAprilTagPoseEstimation(stream, VPI_BACKEND_CPU, detections,
                                    intrinsics, tagSize, poses);

    vpiStreamSync(stream);

    VPIArrayData detections_data;
    vpiArrayLockData(detections, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                     &detections_data);

    VPIArrayData poses_data;
    vpiArrayLockData(poses, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
                     &poses_data);

    VPIPose* p = (VPIPose*)poses_data.buffer.aos.data;
    if ((*detections_data.buffer.aos.sizePointer != 0)) {
      std::cout << "Got detections"
                << "\n";
      frc::Transform3d camera_relitive_position =
          Transform3dFromMatrix(p->transform);
      PrintTransform3d(camera_relitive_position);
    }

    vpiArrayUnlock(detections);
    vpiArrayUnlock(poses);
  }

  vpiStreamDestroy(stream);
  vpiImageDestroy(input);
  vpiArrayDestroy(detections);
  vpiArrayDestroy(poses);
  vpiPayloadDestroy(payload);
}
