#include <src/camera/cv_camera.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

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
  std::ifstream file("/bos/constants/usb_camera0_intrinsics.json");
  json intrinsics;
  file >> intrinsics;

  cv::Mat camera_matrix = camera_matrix_from_json(intrinsics);
  cv::Mat distortion_coefficients =
      distortion_coefficients_from_json(intrinsics);

  camera::CscoreStreamer raw_streamer("raw_stream", 4971, 30, 1080, 1080);
  camera::CscoreStreamer undistorted_streamer("undistorted_stream", 4971, 30,
                                              1080, 1080);

  camera::Camera camera = camera::SelectCamera();
  camera::CVCamera cap(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));
  cv::Mat frame;

  while (true) {
    cap.GetFrame(frame);
    raw_streamer.WriteFrame(frame);

    cv::Mat undistorted;
    cv::undistort(frame, undistorted, camera_matrix, distortion_coefficients);

    undistorted_streamer.WriteFrame(undistorted);
  }
}
