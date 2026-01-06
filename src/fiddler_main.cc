#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "localization/position_sender.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/localization/get_field_relitive_position.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/utils/timer.h"

using json = nlohmann::json;

auto main() -> int {
  utils::StartNetworktables();

  camera::CameraSource back_left_camera = camera::CameraSource(
      "back_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::FIDDLER_USB0].pipeline)));

  camera::CameraSource back_right_camera = camera::CameraSource(
      "back_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::FIDDLER_USB1].pipeline)));

  std::thread usb0_thread(
      localization::run_localization, std::ref(back_left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          640, 480,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB0]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB0].extrinsics_path,
      4971, false);

  std::thread usb1_thread(
      localization::run_localization, std::ref(back_right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          1280, 720,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::FIDDLER_USB1]
                  .intrinsics_path)),
      camera::camera_constants[camera::Camera::FIDDLER_USB1].extrinsics_path,
      4972, false);

  usb0_thread.join();

  return 0;
}
