#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "localization/position_sender.h"
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/utils/timer.h"
#include "src/yolo/model_constants.h"
#include "src/yolo/yolo.h"
#include "src/gamepiece/gamepiece.h"

using json = nlohmann::json;

int main() {
  utils::StartNetworktables();

  std::shared_ptr<camera::CameraSource> back_left_camera = std::make_shared<camera::CameraSource>(
      "back_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB0].pipeline)));

  std::shared_ptr<camera::CameraSource> back_right_camera = std::make_shared<camera::CameraSource>(
      "back_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB1].pipeline)));

  std::thread usb0_thread(
      localization::run_localization, back_left_camera,
      std::make_unique<localization::GPUAprilTagDetector>(
          640, 480,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::USB0].intrinsics_path)),
      camera::camera_constants[camera::Camera::USB0].extrinsics_path, 4971,
      false);

  std::thread usb1_thread(
      localization::run_localization, back_right_camera,
      std::make_unique<localization::GPUAprilTagDetector>(
          1280, 720,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::USB1].intrinsics_path)),
      camera::camera_constants[camera::Camera::USB1].extrinsics_path, 4972,
      false);

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> coral_table =
      inst.GetTable("Orin/Gamepiece/coral");
  std::shared_ptr<nt::NetworkTable> algae_table =
      inst.GetTable("Orin/Gamepiece/algae");
  nt::StructTopic<frc::Pose2d> coral_topic =
      coral_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose2d> algae_topic =
      algae_table->GetStructTopic<frc::Pose2d>("Pose");

  yolo::ModelInfo model_info = yolo::models[yolo::Model::COLOR];
  yolo::Yolo color_model(model_info.path, model_info.color);

  std::thread usb0_gamepiece_thread(gamepiece::run_gamepiece_detect, std::ref(color_model), std::ref(model_info.class_names), back_left_camera, std::ref(coral_topic), std::ref(algae_topic), utils::read_intrinsics(camera::camera_constants[camera::Camera::USB1].intrinsics_path),
      utils::read_extrinsics(camera::camera_constants[camera::Camera::USB1].extrinsics_path), true);

  usb0_gamepiece_thread.join();

  return 0;
}
