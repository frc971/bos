#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/realsense_camera.h"
#include "src/yolo/yolo.h"

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/ninthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath);
  camera::RealSenseCamera rs_camera;
  const int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {"Algae", "ALGAE", "Coral", "CORAL"};
  while (true) {
    cv::Mat mat;
    rs_camera.getFrame(mat);
    if (mat.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    model.Postprocess(mat, bboxes, confidences, class_ids);
    yolo::Yolo::DrawDetections(mat, bboxes, class_ids, confidences,
                               class_names);
    const cv::Rect& b = bboxes[0];
    const double center_x = b.x + b.width / 2.0;
    const double yaw_deg = (mat.cols / 2.0 - center_x) / mat.cols * 69;
    std::cout << "Yaw_deg:" << yaw_deg << std::endl;
    cv::imshow("Test detections", mat);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}
