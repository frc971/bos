#include "src/yolo/yolo.h"
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

const bool TEST_COLLECTED = false;
const bool COLOR = true;
const int MAX_DETECTIONS = 6;

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/ninthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath, COLOR);
  camera::CVCamera camera =
      camera::CVCamera(cv::VideoCapture(camera::SelectCamera()));
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  std::vector<std::string> class_names = {
      "Algae", "ALGAE", "Coral",
      "CORAL"};  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
  if (TEST_COLLECTED) {
    for (const auto& entry : std::filesystem::directory_iterator(
             std::string(std::getenv("HOME")) + "/Documents/collected_imgs")) {
      cv::Mat mat = cv::imread(entry.path().string());
      model.Postprocess(mat.rows, mat.cols, model.RunModel(mat), bboxes, confidences, class_ids);
      yolo::Yolo::DrawDetections(mat, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", mat);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  } else {
    cv::Mat frame;
    while (true) {
      camera.GetFrame(frame);
      if (frame.empty()) {
        std::cout << "Couldn't fetch frame properly" << std::endl;
        return 1;
      }
      model.Postprocess(frame.rows, frame.cols, model.RunModel(frame), bboxes, confidences, class_ids);
      yolo::Yolo::DrawDetections(frame, bboxes, class_ids, confidences,
                                 class_names);
      cv::imshow("Test detections", frame);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
  }
}
