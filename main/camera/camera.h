#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
namespace Camera {

typedef struct CameraInfo {
  std::string pipeline;
  std::string name;
} camera_info_t;

struct kCameras {
  CameraInfo gstreamer1_30fps = {
      .pipeline =
          "nvarguscamerasrc sensor-id=0 ! "
          "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, format=NV12 ! "
          "nvvidconv ! "
          "video/x-raw, format=BGRx ! "
          "appsink",

      .name = "Gstreamer #1 30fps"};

  CameraInfo gstreamer2_30fps = {
      .pipeline =
          "nvarguscamerasrc sensor-id=1 !"
          "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, format=NV12 ! "
          "nvvidconv ! "
          "video/x-raw, format=BGRx ! "
          "appsink",

      .name = "Gstreamer #2 30fps"};
};

const kCameras CAMERAS;

inline void WarmupCamera(std::string pipeline) {
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
}

class Camera {
public:
  Camera(camera_info_t info);
  ~Camera();
  void getFrame(cv::Mat &mat);

private:
  camera_info_t info_;
  cv::VideoCapture cap_;
};

} // namespace Camera

#endif // CAMERA_H
