// https://gist.github.com/SteveRuben/1a15909e384b582c51b5
#ifndef IMX296_CAMERA_H
#define IMX296_CAMERA_H
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
namespace Camera {

typedef struct CameraInfo {
  std::string pipeline;
  std::string name;
  std::string intrinsics_path;
  std::string extrinsics_path;
  int id;
} camera_info_t;

struct kCameras {
  CameraInfo gstreamer1_30fps = {
      .pipeline =
          "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
          "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
          "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
          "format=NV12 ! "
          "nvvidconv ! "
          "video/x-raw, format=BGRx ! "
          "queue ! "
          "appsink",
      .name = "Gstreamer #0 30fps",
      .intrinsics_path = "constants/camera0_intrinsics.json",
      .extrinsics_path = "constants/camera0_extrinsics.json",
      .id = 0};

  CameraInfo gstreamer2_30fps = {
      .pipeline =
          "nvarguscamerasrc sensor-id=1 !"
          "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
          "format=NV12 ! "
          "nvvidconv ! "
          "video/x-raw, format=BGRx ! "
          "queue ! "
          "appsink",
      .name = "Gstreamer #1 30fps",
      .intrinsics_path = "constants/camera1_intrinsics.json",
      .extrinsics_path = "constants/camera1_extrinsics.json",
      .id = 1};
};

const kCameras CAMERAS;

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

}  // namespace Camera

#endif  // IMX296_CAMERA_H
