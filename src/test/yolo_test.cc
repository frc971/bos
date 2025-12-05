#include "src/yolo/yolo.h"
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/ninthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath, 3);
  camera::CVCamera camera =
      camera::CVCamera(cv::VideoCapture(camera::SelectCamera()));
  const int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {
      "Algae", "ALGAE", "Coral",
      "CORAL"};  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
  const bool test_collected = false;
  cv::Mat color;
  if (test_collected) {
    for (const auto& entry : std::filesystem::directory_iterator(
             std::string(std::getenv("HOME")) + "/Documents/collected_imgs")) {
      cv::Mat mat = cv::imread(entry.path().string());
      model.Postprocess(mat, bboxes, confidences, class_ids);
      yolo::Yolo::DrawDetections(mat, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", mat);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  } else {
    while (true) {
      camera.GetFrame(color);
      if (color.empty()) {
        std::cout << "Couldn't fetch frame properly" << std::endl;
        return 1;
      }
      model.Postprocess(color, bboxes, confidences, class_ids);
      yolo::Yolo::DrawDetections(color, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", color);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  }
}
