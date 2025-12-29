#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/utils/nt_utils.h"
#include "src/utils/timer.h"
#include "src/yolo/model_constants.h"
#include "src/yolo/yolo.h"
#include "src/gamepiece/gamepiece.h"
#include "src/utils/camera_utils.h"

using json = nlohmann::json;

void run_estimator(const int frame_width, const int frame_height,
                   camera::CameraSource& source, std::string intrinsics,
                   std::string extrinsics, uint port) {

  localization::TagEstimator tag_estimator(frame_width, frame_height,
                                           intrinsics, extrinsics);
  localization::PositionSender position_sender(source.GetName());

  camera::CscoreStreamer streamer(source.GetName(), 4971, 30, 1080, 1080);

  while (true) {
    utils::Timer timer(source.GetName(), false);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(timestamped_frame.frame,
                               timestamped_frame.timestamp);
    position_sender.Send(estimates, timer.Stop());
  }
}

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

  std::thread usb0_localization_thread(
      run_estimator, 640, 480, back_left_camera,
      camera::camera_constants[camera::Camera::USB0].intrinsics_path,
      camera::camera_constants[camera::Camera::USB0].extrinsics_path, 4971);

  std::thread usb1_localization_thread(
      run_estimator, 1280, 720, back_right_camera,
      camera::camera_constants[camera::Camera::USB1].intrinsics_path,
      camera::camera_constants[camera::Camera::USB1].extrinsics_path, 4971);
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

  usb1_localization_thread.join();

  return 0;
}
