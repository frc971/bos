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
  while (true) {
    cv::Mat color_mat;
    cv::Mat depth_mat;
    rs_camera.getFrame(color_mat, depth_mat);
    if (color_mat.empty() || depth_mat.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    model.Postprocess(color_mat, bboxes, confidences, class_ids);
    const int c_x = bboxes[0].x + bboxes[0].width / 2;
    const int c_y = bboxes[0].y + bboxes[0].height / 2;
  }
}
