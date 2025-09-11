#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "main/camera/camera.h"
#include "main/camera/streamer.h"

using json = nlohmann::json;

cv::Mat camera_matrix_from_json(json intrinsics) {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);

  return camera_matrix;
}

cv::Mat distortion_coefficients_from_json(json intrinsics) {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

void warmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}
int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are logging?\n";
  int camera_id;
  std::cin >> camera_id;

  Camera::CameraInfo camera_info;

  switch (camera_id) {
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

  std::ifstream file(camera_info.intrinsics_path);
  json intrinsics;
  file >> intrinsics;

  cv::Mat camera_matrix = camera_matrix_from_json(intrinsics);
  cv::Mat distortion_coefficients =
      distortion_coefficients_from_json(intrinsics);

  Camera::Streamer raw_streamer(4971, true);
  Camera::Streamer undistorted_streamer(4972, true);

  Camera::Camera camera(camera_info);
  cv::Mat frame;

  while (true) {
    camera.getFrame(frame);
    raw_streamer.WriteFrame(frame);

    cv::Mat undistorted;
    cv::undistort(frame, undistorted, camera_matrix, distortion_coefficients);

    undistorted_streamer.WriteFrame(undistorted);
  }
}
