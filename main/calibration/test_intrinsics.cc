#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

using json = nlohmann::json;

cv::Mat camera_matrix_from_json(json intrinsics) {
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
    intrinsics["fx"], 0, intrinsics["cx"],
    0, intrinsics["fy"], intrinsics["cy"],
    0, 0, 1);

  return camera_matrix;
}

cv::Mat distortion_coefficients_from_json(json intrinsics) {
  cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << 
    intrinsics["k1"], intrinsics["k2"], intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

void warmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}
int main() {
  std::ifstream file("intrinsics.json");
  json intrinsics;
  file >> intrinsics;
  cv::Mat camera_matrix = camera_matrix_from_json(intrinsics);
  cv::Mat distortion_coefficients = distortion_coefficients_from_json(intrinsics);

  std::string pipeline =
      "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
      "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
      "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
      "format=NV12 ! "
      "nvvidconv ! "
      "video/x-raw, format=BGRx ! "
      "appsink";

  warmupCamera(pipeline);
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  cv::Mat frame;
  cv::Mat gray;
  while (true) {
    cap >> frame;
    cv::Mat undistorted;

    cv::undistort(frame, undistorted, camera_matrix, distortion_coefficients);
    cv::imshow("raw", frame);
    cv::imshow("undistorted", undistorted);

    char c = (char)cv::waitKey(1);
    if (c == 'q' || c == 27) {
      break;
    }
  }
}
