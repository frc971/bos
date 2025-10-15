#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"
#include <fmt/format.h>
namespace camera {

typedef struct CameraInfo {
  std::string pipeline;
  std::string name;  // name in the networktables
  std::string intrinsics_path;
  std::string extrinsics_path;
  int id;
} camera_info_t;

static std::string Imx296PipelineTemplate(int sensor_id, int framerate) {
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

const CameraInfo gstreamer1_30fps = {
    .pipeline = Imx296PipelineTemplate(0, 30),
    .name = "camera_1",
    .intrinsics_path = "constants/camera0_intrinsics.json",
    .extrinsics_path = "constants/camera0_extrinsics.json",
    .id = 0};

const CameraInfo gstreamer2_30fps = {
    .pipeline = Imx296PipelineTemplate(1, 30),
    .name = "camera_2",
    .intrinsics_path = "constants/camera1_intrinsics.json",
    .extrinsics_path = "constants/camera1_extrinsics.json",
    .id = 1};

const CameraInfo gstreamer1_60fps = {
    .pipeline = Imx296PipelineTemplate(0, 60),
    .name = "camera_1",
    .intrinsics_path = "constants/camera0_intrinsics.json",
    .extrinsics_path = "constants/camera0_extrinsics.json",
    .id = 0};

const CameraInfo gstreamer2_60fps = {
    .pipeline = Imx296PipelineTemplate(1, 60),
    .name = "camera_2",
    .intrinsics_path = "constants/camera1_intrinsics.json",
    .extrinsics_path = "constants/camera1_extrinsics.json",
    .id = 1};

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
