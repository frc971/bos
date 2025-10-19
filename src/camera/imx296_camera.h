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

const std::string gstreamer1_30fps =
    "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
    "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
    "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
    "format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "queue ! "
    "appsink";

const std::string gstreamer2_30fps =
    "nvarguscamerasrc sensor-id=1 !"
    "video/x-raw(memory:NVMM), width=1456, height=1088, "
    "framerate=30/1, "
    "format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "queue ! "
    "appsink";

const std::string gstreamer1_60fps =
    "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
    "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
    "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=60/1, "
    "format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "queue ! "
    "appsink";

const std::string gstreamer2_60fps =
    "nvarguscamerasrc sensor-id=1 !"
    "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=60/1, "
    "format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "queue ! "
    "appsink";

inline void WarmupIMX296Camera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}

}  // namespace camera
