#include <iostream>
#include "camera/realsense_camera.h"
#include "yolo/yolo.h"

int main() {
  yolo::Yolo model("/bos/main/yolo/yolo11n.pt");
  camera::RealSenseCamera rs_camera;
  cv::Mat mat;
  rs_camera.getFrame(mat);
  const std::vector<float> maybe_softmax_results = model.RunModel(mat);
  for (const float& maybe_softmax_val : maybe_softmax_results) {
    std::cout << maybe_softmax_val << std::endl;
  }
}
