#include <third_party/971apriltag/971apriltag.h>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

using json = nlohmann::json;
frc971::apriltag::CameraMatrix camera_matrix_from_json(json intrinsics) {
  frc971::apriltag::CameraMatrix camMat = {.fx = intrinsics["fx"],
                                           .cx = intrinsics["cx"],
                                           .fy = intrinsics["fy"],
                                           .cy = intrinsics["cy"]};
  return camMat;
}

frc971::apriltag::DistCoeffs
distortion_coefficients_from_json(json intrinsics) {
  frc971::apriltag::DistCoeffs dist = {.k1 = intrinsics["k1"],
                                       .k2 = intrinsics["k2"],
                                       .p1 = intrinsics["p1"],
                                       .p2 = intrinsics["p2"],
                                       .k3 = intrinsics["k3"]};

  return dist;
}

struct detectorstruct {
  frc971::apriltag::GpuDetector *gpu_detector_;
  apriltag_detector_t *apriltag_detector_;
  frc971::apriltag::CameraMatrix camera_matrix;
  frc971::apriltag::DistCoeffs dist_coeffs;
};

apriltag_detector_t *maketagdetector(apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = false;

  return tag_detector;
}

float tag_size = 6.5; // meters
std::vector<cv::Point3f> objectPoints = {{-tag_size / 2, tag_size / 2, 0},
                                         {tag_size / 2, tag_size / 2, 0},
                                         {tag_size / 2, -tag_size / 2, 0},
                                         {-tag_size / 2, -tag_size / 2, 0}};

void warmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}
int main() {
  std::cout << "creategpudetector " << std::endl;
  detectorstruct detector;

  std::ifstream file("intrinsics.json");
  json intrinsics;
  file >> intrinsics;

  detector.camera_matrix = camera_matrix_from_json(intrinsics);
  detector.dist_coeffs = distortion_coefficients_from_json(intrinsics);
  detector.apriltag_detector_ = maketagdetector(tag36h11_create());
  detector.gpu_detector_ = new frc971::apriltag::GpuDetector(
      1456, 1088, detector.apriltag_detector_, detector.camera_matrix,
      detector.dist_coeffs);
  int cols = detector.gpu_detector_->width();
  int rows = detector.gpu_detector_->height();
  std::cout << cols << "\n" << rows;

  std::string pipeline =
      "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
      "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
      "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=60/1, "
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
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    detector.gpu_detector_->DetectGrayHost((unsigned char *)gray.ptr());
    const zarray_t *detections = detector.gpu_detector_->Detections();

    cv::imshow("Camera", gray);
    if (zarray_size(detections)) {
      for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t *gpu_detection;
        zarray_get(detections, i, &gpu_detection);
        std::printf("tag: %f, %f id: %d\n", gpu_detection->c[0],
                    gpu_detection->c[1], gpu_detection->id);
        cv::Point point(gpu_detection->c[0], gpu_detection->c[1]);
        cv::circle(frame, point, 3, cv::Scalar(0, 0, 255), cv::FILLED);

        std::vector<cv::Point2f> imagePoints;
        for (int i = 0; i < 4; ++i) {
          imagePoints.emplace_back(gpu_detection->p[i][0],
                                   gpu_detection->p[i][1]);
        }
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
        cv::Mat tvec =
            cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector
        cv::Mat cameraMatrix =
            (cv::Mat_<double>(3, 3) << detector.camera_matrix.fx, 0,
             detector.camera_matrix.cx, 0, detector.camera_matrix.fy,
             detector.camera_matrix.cy, 0, 0, 1);
        cv::Mat distCoeffs =
            cv::Mat::zeros(4, 1, CV_64FC1); // vector of distortion coefficients
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,
                     tvec);

        for (int i = 0; i < rvec.size[0]; i++){
          rvec.ptr<double>()[i] = rvec.ptr<double>()[i] * (180 / M_PI);
        }
        std::cout << "--- Pose Estimation Results ---" << std::endl;
        std::cout << "Rotation Vector (rvec):\n" << rvec << std::endl;
        std::cout << "Translation Vector (tvec):\n" << tvec << std::endl;
        std::cout << "-------------------------------" << std::endl;
        break;
      }
    }
    cv::imshow("Camera", frame);

    char c = (char)cv::waitKey(1);
    if (c == 'q' || c == 27) {
      break;
    }
  }
}
