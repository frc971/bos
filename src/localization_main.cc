#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/nt_utils.h"

using json = nlohmann::json;

void run_estimator(std::string name, const int frame_width,
                   const int frame_height,
                   std::unique_ptr<camera::CVCamera> cap, json intrinsics,
                   json extrinsics,
                   localization::PositionSender& position_sender, int port) {

  localization::TagEstimator tag_estimator(frame_width, frame_height,
                                           intrinsics, extrinsics);

  camera::CscoreStreamer streamer(name, port, 30, 480, 480, false);

  cv::Mat frame;
  while (true) {
    cap->GetFrame(frame);
    streamer.WriteFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(frame);
    position_sender.Send(estimates);
  }
}

int main() {

  NTUtils::start_networktables();

  localization::PositionSender position_sender(true);

  std::thread usb0_thread(
      run_estimator, "back_left", 640, 480,
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB0].pipeline)),
      camera::ICamera::read_intrinsics(
          camera::camera_constants[camera::Camera::USB0].intrinsics_path),
      camera::ICamera::read_extrinsics(
          camera::camera_constants[camera::Camera::USB0].extrinsics_path),
      std::ref(position_sender), 4971);

  std::thread usb1_thread(
      run_estimator, "back_right", 1280, 720,
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB1].pipeline)),
      camera::ICamera::read_intrinsics(
          camera::camera_constants[camera::Camera::USB1].intrinsics_path),
      camera::ICamera::read_extrinsics(
          camera::camera_constants[camera::Camera::USB1].extrinsics_path),
      std::ref(position_sender), 4972);

  usb1_thread.join();

  return 0;
}
