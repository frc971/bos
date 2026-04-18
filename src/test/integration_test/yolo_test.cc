#include "src/yolo/yolo.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"
#include "src/utils/timer.h"
#include "src/yolo/model_constants.h"

const int MAX_DETECTIONS = 10;

auto main() -> int {
  yolo::ModelInfo model_info = yolo::models[yolo::Model::COLOR];
  yolo::Yolo model(model_info.path, model_info.color, true);
  std::unique_ptr<camera::ICamera> camera =
      camera::SelectCameraConfig(camera::GetCameraConstants());
  camera::timestamped_frame_t timestamped_frame;
  camera->GetFrame(&timestamped_frame);

  camera::CscoreStreamer streamer("yolo_test", 5801, 30,
                                  timestamped_frame.frame);

  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);

  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
  while (true) {
    utils::Timer timer("yolo");
    camera->GetFrame(&timestamped_frame);
    cv::Mat& frame = timestamped_frame.frame;
    if (frame.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    std::vector<float> detections = model.RunModel(frame);
    model.Postprocess(frame.rows, frame.cols, detections, bboxes, confidences,
                      class_ids);
    yolo::Yolo::DrawDetections(frame, bboxes, class_ids, confidences,
                               model_info.class_names);
    std::cout << "Object angle: "
              << yolo::Yolo::GetObjectAngle(
                     (detections[0] + detections[2]) / 2.0,
                     std::numbers::pi * (3.0 / 4.0))
              << "\n";
    streamer.WriteFrame(frame);
  }
}
