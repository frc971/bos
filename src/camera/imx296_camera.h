#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"
namespace camera {

typedef struct CameraInfo {
  std::string pipeline;
  std::string name;  // name in the networktables
  std::string intrinsics_path;
  std::string extrinsics_path;
  int id;
} camera_info_t;

const int num_cameras = 2;

static inline std::string IMX296Pipeline(int sensor_id, int framerate) {
  std::string pipeline = 
    "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " aelock=true exposuretimerange=\"100000 "
    "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
    "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=" + std::to_string(framerate) + "/1, "
    "format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "queue ! "
    "appsink";
  return pipeline;
}

static inline CameraInfo IMX296Template(int camera_id, int framerate) {
  camera::CameraInfo camera_info = {
    .pipeline = camera::IMX296Pipeline(camera_id, framerate),
    .name = "camera_" + std::to_string(camera_id),
    .intrinsics_path = "constants/camera" + std::to_string(camera_id) + "_intrinsics.json",
    .extrinsics_path = "constants/camera" + std::to_string(camera_id) + "_extrinsics.json",
    .id = camera_id};
  return camera_info;
}

inline void WarmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}

class IMX296Camera : Camera {
 public:
  IMX296Camera(camera_info_t info);
  ~IMX296Camera();
  void GetFrame(cv::Mat& mat);

 private:
  camera_info_t info_;
  cv::VideoCapture cap_;
};

}  // namespace camera
