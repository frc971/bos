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
#include "src/localization/tag_estimator.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/utils/timer.h"

using json = nlohmann::json;

void run_estimator(const int frame_width, const int frame_height,
                   camera::CameraSource& source, std::string intrinsics,
                   std::string extrinsics, uint port, bool verbose = false) {

  localization::GPUAprilTagDetector detector(
      frame_width, frame_height, utils::read_intrinsics(intrinsics));
  localization::PositionSender position_sender(source.GetName());

  camera::CscoreStreamer streamer(source.GetName(), port, 30, 1080, 1080);

  nlohmann::json extrinsics_json = utils::read_extrinsics(extrinsics);
  while (true) {
    utils::Timer timer(source.GetName(), verbose);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> estimates =
        localization::GetFeildRelitivePosition(
            detector.GetTagDetections(timestamped_frame), extrinsics_json);
    position_sender.Send(estimates, timer.Stop());
  }
}

int main() {
  utils::StartNetworktables();

  camera::Camera config = camera::SelectCameraConfig();
  camera::CameraSource source("stress_test_camera",
                              camera::GetCameraStream(config));
  cv::Mat frame = source.GetFrame();

  std::thread usb0_thread(
      run_estimator, frame.cols, frame.rows, std::ref(source),
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path, 4971, false);

  std::thread usb1_thread(
      run_estimator, frame.cols, frame.rows, std::ref(source),
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path, 4972, false);

  std::thread usb2_thread(
      run_estimator, frame.cols, frame.rows, std::ref(source),
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path, 4973, false);

  std::thread usb3_thread(
      run_estimator, frame.cols, frame.rows, std::ref(source),
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path, 4974, false);

  std::thread usb4_thread(
      run_estimator, frame.cols, frame.rows, std::ref(source),
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path, 4974, true);

  usb0_thread.join();

  return 0;
}
