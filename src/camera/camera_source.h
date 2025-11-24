#pragma once

#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>
#include "src/camera/camera.h"
#include "third_party/circular_buffer/circular_buffer.h"
namespace camera {

typedef struct Frame {
  cv::Mat& mat;
  double timestamp;
} frame_t;

class CameraSource {
 public:
  CameraSource(std::unique_ptr<ICamera> camera);
  frame_t Get();

 private:
  std::unique_ptr<ICamera> camera_;
  CircularBuffer<frame_t> buffer_;
  std::thread thread_;
};

}  // namespace camera
