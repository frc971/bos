#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "main/camera/camera.h"

using json = nlohmann::json;

int main() {

  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are logging?\n";
  int camera_id;
  std::cin >> camera_id;

  Camera::CameraInfo camera_info;

  switch (camera_id){
  case 0:
    camera_info = Camera::CAMERAS.gstreamer1_30fps;
    break;
  case 1:
    camera_info = Camera::CAMERAS.gstreamer2_30fps;
    break;
  default:
    std::cout << "Invalid ID! Only 0 or 1" << std::endl;
    return 0;
  }

  json intrinsics;
  std::ifstream intrinsics_file(camera_info.intrinsics_path);
  if (!intrinsics_file.is_open()) {
      std::cerr << "Error: Cannot open intrinsics file: " 
                << camera_info.intrinsics_path << std::endl;
  } else {
      intrinsics_file >> intrinsics;
  }

  std::cout << "What tag are we calibrating against?\n";
  int tag_id;
  std::cin >> tag_id;

  Camera::Camera camera(camera_info);
  cv::Mat frame;

  while (true){
    camera.getFrame(frame);
    // std::vector<PoseEstimator::position_estimate_t> estimates = estimator.Estimate(frame);
    // sender.Send(estimates);
  }


}
