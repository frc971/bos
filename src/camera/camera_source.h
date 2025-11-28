#pragma once

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>
#include "src/camera/camera.h"
namespace camera {

typedef struct Frame {
  cv::Mat mat;
  double timestamp;
} frame_t;

class CameraSource {
 public:
  CameraSource(int width, int height, int channels, int image_type,
               std::unique_ptr<ICamera> camera, int length);
  frame_t Get();

 private:
  int width_;
  int height_;
  int channels_;
  int image_type_;
  size_t image_size_;
  std::unique_ptr<ICamera> camera_;
  int length_;
  int head_;
  uint8_t* buffer_;
  double* timestamp_;
  std::thread thread_;
};

}  // namespace camera
