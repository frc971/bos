#include "src/yolo/yolo.h"
#include <filesystem>
#include <iostream>
#include <numbers>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/camera_constants.h"
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
  auto camera = camera::Camera::DEFAULT_USB0;
  camera = camera::SelectCamera();
  camera::CVCamera cap = camera::CVCamera(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("yolo_test", 4971, 30));

  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);

  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
  std::vector<std::string> class_names = {"object"};
  while (true) {
    cv::Mat frame;
    utils::Timer timer("yolo");
    cap.GetFrame(frame);
    if (frame.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    std::vector<float> detections = model.RunModel(frame);
    model.Postprocess(frame.rows, frame.cols, detections, bboxes, confidences,
                      class_ids);
    yolo::Yolo::DrawDetections(frame, bboxes, class_ids, confidences,
                               class_names);
    std::cout << "yaw angle: "
              << yolo::Yolo::GetObjectAngle(
                     (detections[yolo::OuputBufferIndexes::X1] +
                      detections[yolo::OuputBufferIndexes::X2]) /
                         2.0,
                     std::numbers::pi * (3.0 / 4.0))
              << "\n";

    const double pitch_angle =
        yolo::Yolo::GetObjectAngle((detections[yolo::OuputBufferIndexes::Y1] +
                                    detections[yolo::OuputBufferIndexes::Y2]) /
                                       2.0,
                                   std::numbers::pi * (3.0 / 4.0));
    std::cout << "pith angle: " << pitch_angle << "\n";

    const double estimated_distance =
        yolo::Yolo::GetObjectDistance(std::abs(pitch_angle), 1.0);
    std::cout << "estimated_distance: " << estimated_distance << "\n";

    streamer.WriteFrame(frame);
  }
}
