// https://gist.github.com/SteveRuben/1a15909e384b582c51b5
#ifndef IMX296_CAMERA_H
#define IMX296_CAMERA_H
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
namespace camera {

typedef struct CameraInfo {
  std::string pipeline;
  std::string name;  // name in the networktables
  std::string intrinsics_path;
  std::string extrinsics_path;
  int id;
} camera_info_t;

const CameraInfo gstreamer1_30fps = {
    .pipeline =
        "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
        "format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "queue ! "
        "appsink",
    .name = "camera_1",
    .intrinsics_path = "constants/camera0_intrinsics.json",
    .extrinsics_path = "constants/camera0_extrinsics.json",
    .id = 0};

const CameraInfo gstreamer2_30fps = {
    .pipeline =
        "nvarguscamerasrc sensor-id=1 !"
        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
        "format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "queue ! "
        "appsink",
    .name = "camera_2",
    .intrinsics_path = "constants/camera1_intrinsics.json",
    .extrinsics_path = "constants/camera1_extrinsics.json",
    .id = 1};

const CameraInfo gstreamer1_60fps = {
    .pipeline =
        "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=60/1, "
        "format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "queue ! "
        "appsink",
    .name = "camera_1",
    .intrinsics_path = "constants/camera0_intrinsics.json",
    .extrinsics_path = "constants/camera0_extrinsics.json",
    .id = 0};

const CameraInfo gstreamer2_60fps = {
    .pipeline =
        "nvarguscamerasrc sensor-id=1 !"
        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=60/1, "
        "format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "queue ! "
        "appsink",
    .name = "camera_2",
    .intrinsics_path = "constants/camera1_intrinsics.json",
    .extrinsics_path = "constants/camera1_extrinsics.json",
    .id = 1};

inline void WarmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}

class IMX296Camera {
 public:
  IMX296Camera(camera_info_t info);
  ~IMX296Camera();
  void getFrame(cv::Mat& mat);

 private:
  camera_info_t info_;
  cv::VideoCapture cap_;
};

}  // namespace camera

#endif  // IMX296_CAMERA_H
