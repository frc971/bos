#include <iostream>
#include "main/camera/realsense_camera.h"
#include "main/yolo/yolo.h"

int main() {
  std::cout << "Importing model" << std::endl;
  yolo::Yolo model("/bos/main/yolo/yolo11n.engine");
  camera::RealSenseCamera rs_camera;
  cv::Mat mat;
  rs_camera.getFrame(mat);
  if (mat.empty()) {
    std::cout << "Couldn't fetch frame properly" << std::endl;
    return 1;
  }
  const std::vector<float> maybe_softmax_results = model.RunModel(mat);
  for (const float& maybe_softmax_val : maybe_softmax_results) {
    std::cout << maybe_softmax_val << std::endl;
  }
}
