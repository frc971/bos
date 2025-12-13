#include "src/yolo/yolo.h"
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"
#include "src/utils/timer.h"

const int MAX_DETECTIONS = 10;

int main() {
  std::string model_path = "/bos/models/model.engine";
  std::cout << "Model path\n";
  std::cin >> model_path;

  yolo::Yolo model(model_path, true, true);
  auto camera = camera::SelectCamera();
  camera::CVCamera cap = camera::CVCamera(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("yolo_test", 4971, 30));

  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);

  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
  std::vector<std::string> class_names = {"fork"};
  while (true) {
    cv::Mat frame;
    utils::Timer timer("yolo");
    cap.GetFrame(frame);
    if (frame.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    model.Postprocess(frame.rows, frame.cols, model.RunModel(frame), bboxes,
                      confidences, class_ids);
    yolo::Yolo::DrawDetections(frame, bboxes, class_ids, confidences,
                               class_names);
    streamer.WriteFrame(frame);
  }
}
