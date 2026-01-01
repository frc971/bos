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

using json = nlohmann::json;

void run_yolo(const int frame_width, const int frame_height,
              yolo::ModelInfo& model_info, camera::CameraSource& source,
              const std::string& extrinsics, uint port) {
  yolo::Yolo model(model_info.path, model_info.color, true);

  camera::CscoreStreamer streamer(source.GetName(), 4971, 30, 1080, 1080);

  std::vector<cv::Rect> bboxes(6);
  std::vector<float> confidences(6);
  std::vector<int> class_ids(6);

  while (true) {
    camera::timestamped_frame_t timestamped_frame = source.Get();

    std::vector<float> detections = model.RunModel(timestamped_frame.frame);
    model.Postprocess(timestamped_frame.frame.rows,
                      timestamped_frame.frame.cols, detections, bboxes,
                      confidences, class_ids);

    yolo::Yolo::DrawDetections(timestamped_frame.frame, bboxes, class_ids,
                               confidences, model_info.class_names);
    streamer.WriteFrame(timestamped_frame.frame);
  }
}

auto main() -> int {
  utils::StartNetworktables();

  camera::CameraSource back_left_camera(
      "back_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB0].pipeline)));

  camera::CameraSource back_right_camera(
      "back_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB1].pipeline)));

  std::thread usb0_thread(
      localization::run_localization, std::ref(back_left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          640, 480,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::USB0].intrinsics_path)),
      camera::camera_constants[camera::Camera::USB0].extrinsics_path, 4971,
      false);

  std::thread usb1_thread(
      localization::run_localization, std::ref(back_right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          1280, 720,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::USB1].intrinsics_path)),
      camera::camera_constants[camera::Camera::USB1].extrinsics_path, 4972,
      false);

  usb0_thread.join();

  return 0;
}
