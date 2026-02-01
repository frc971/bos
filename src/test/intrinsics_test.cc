#include <src/camera/cv_camera.h>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT

using json = nlohmann::json;

auto camera_matrix_from_json(json intrinsics) -> cv::Mat {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);

  return camera_matrix;
}

auto distortion_coefficients_from_json(json intrinsics) -> cv::Mat {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

void warmupCamera(const std::string& pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  camera::Camera config =
      camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));
  camera::CameraSource source("stress_test_camera",
                              camera::GetCameraStream(config));
  cv::Mat frame = source.GetFrame();

  std::ifstream intrinsics_file(
      camera::camera_constants[config].intrinsics_path);
  json intrinsics;
  intrinsics_file >> intrinsics;

  cv::Mat camera_matrix = camera_matrix_from_json(intrinsics);
  cv::Mat distortion_coefficients =
      distortion_coefficients_from_json(intrinsics);

  camera::CscoreStreamer raw_streamer("raw_stream", 4971, 30, 1080, 1080);
  camera::CscoreStreamer undistorted_streamer("undistorted_stream", 4972, 30,
                                              1080, 1080);

  while (true) {
    frame = source.GetFrame();
    raw_streamer.WriteFrame(frame);

    cv::Mat undistorted;
    cv::undistort(frame, undistorted, camera_matrix, distortion_coefficients);

    undistorted_streamer.WriteFrame(undistorted);
  }
}
