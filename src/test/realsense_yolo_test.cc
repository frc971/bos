#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/realsense_camera.h"
#include "src/yolo/yolo.h"

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/gray.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath);
  camera::RealSenseCamera rs_camera;
  const int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {"Algae", "Coral"};
  const bool test_collected = false;
  cv::Mat color;
  if (test_collected) {
    for (const auto& entry : std::filesystem::directory_iterator(
             "/home/nvidia/Documents/collected_imgs")) {
      cv::Mat mat = cv::imread(entry.path().string());
      model.Postprocess(mat, bboxes, confidences, class_ids, false);
      yolo::Yolo::DrawDetections(mat, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", mat);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  } else {
    while (true) {
      rs_camera.GetFrame(color);
      cv::Mat grayscaled;
      cv::cvtColor(color, grayscaled, cv::COLOR_BGR2GRAY);
      if (color.empty()) {
        std::cout << "Couldn't fetch frame properly" << std::endl;
        return 1;
      }
      model.Postprocess(grayscaled, bboxes, confidences, class_ids, true);
      yolo::Yolo::DrawDetections(grayscaled, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", grayscaled);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  }
}
