#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "third_party/971apriltag/971apriltag.h"

using json = nlohmann::json;

frc971::apriltag::CameraMatrix camera_matrix_from_json(json intrinsics) {
  frc971::apriltag::CameraMatrix camera_matrix = {.fx = intrinsics["fx"],
                                                  .cx = intrinsics["cx"],
                                                  .fy = intrinsics["fy"],
                                                  .cy = intrinsics["cy"]};
  return camera_matrix;
}

frc971::apriltag::DistCoeffs distortion_coefficients_from_json(
    json intrinsics) {
  frc971::apriltag::DistCoeffs distortion_coefficients = {
      .k1 = intrinsics["k1"],
      .k2 = intrinsics["k2"],
      .p1 = intrinsics["p1"],
      .p2 = intrinsics["p2"],
      .k3 = intrinsics["k3"]};

  return distortion_coefficients;
}

json read_intrinsics(std::string path) {
  json intrinsics;

  std::ifstream intrinsics_file(path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }
  return intrinsics;
}

int main() {
  auto apriltag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(apriltag_detector_, tag36h11_create(), 1);

  apriltag_detector_->nthreads = 6;
  apriltag_detector_->wp = workerpool_create(apriltag_detector_->nthreads);
  apriltag_detector_->qtp.min_white_black_diff = 4;
  apriltag_detector_->debug = false;

  camera::CscoreStreamer streamer("apriltag_detect_test", 4971, 30, 640, 480,
                                  false);

  auto camera_config = camera::Camera::USB0;

  auto intrinsics =
      read_intrinsics(camera::camera_constants[camera_config].pipeline);

  auto gpu_detector_ = new frc971::apriltag::GpuDetector(
      640, 480, apriltag_detector_, camera_matrix_from_json(intrinsics),
      distortion_coefficients_from_json(intrinsics));
  auto camera = camera::CVCamera(
      cv::VideoCapture(camera::camera_constants[camera_config].pipeline));
  cv::Mat frame;
  cv::Mat gray;

  while (true) {
    camera.GetFrame(frame);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    gpu_detector_->DetectGrayHost((unsigned char*)gray.ptr());
    const zarray_t* detections = gpu_detector_->Detections();

    std::vector<cv::Point2f> imagePoints;
    if (zarray_size(detections)) {
      for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* gpu_detection;
        zarray_get(detections, i, &gpu_detection);
        cv::Point point(gpu_detection->c[0], gpu_detection->c[1]);

        for (int i = 0; i < 4; ++i) {
          imagePoints.emplace_back(gpu_detection->p[i][0],
                                   gpu_detection->p[i][1]);
        }
      }
    }
    for (auto image_point : imagePoints) {
      cv::circle(frame, image_point, 10, cv::Scalar(0, 0, 255));
    }
    streamer.WriteFrame(frame);
  }
}
