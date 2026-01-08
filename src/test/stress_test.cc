#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"
#include "src/localization/get_field_relitive_position.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/run_localization.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/utils/timer.h"

using json = nlohmann::json;

auto main() -> int {
  utils::StartNetworktables();

  camera::Camera config = camera::SelectCameraConfig();
  camera::CameraSource source = camera::CameraSource(
      "stress_test_camera", camera::GetCameraStream(config));
  cv::Mat frame = source.GetFrame();

  // std::bind?
  std::thread usb0_thread(
      localization::run_localization, std::ref(source),
      std::make_unique<localization::GPUAprilTagDetector>(
          frame.cols, frame.rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB0]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB0].extrinsics_path,
      4971, false);

  std::thread usb1_thread(
      localization::run_localization, std::ref(source),
      std::make_unique<localization::GPUAprilTagDetector>(
          frame.cols, frame.rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB0]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB0].extrinsics_path,
      4972, false);

  std::thread usb2_thread(
      localization::run_localization, std::ref(source),
      std::make_unique<localization::GPUAprilTagDetector>(
          frame.cols, frame.rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB0]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB0].extrinsics_path,
      4973, false);

  std::thread usb3_thread(
      localization::run_localization, std::ref(source),
      std::make_unique<localization::GPUAprilTagDetector>(
          frame.cols, frame.rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB0]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB0].extrinsics_path,
      4974, false);

  usb0_thread.join();

  return 0;
}
