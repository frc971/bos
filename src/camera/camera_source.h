#pragma once

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <queue>
#include <string>
#include <thread>
#include "src/camera/camera.h"
namespace camera {

typedef struct TimestampedFrame {
  cv::Mat frame;
  double timestamp;
} timestamped_frame_t;

class CameraSource {
 public:
  CameraSource(std::string name, std::unique_ptr<ICamera> camera);
  timestamped_frame_t Get();
  std::string GetName() const { return name_; }

 private:
  std::string name_;
  std::unique_ptr<ICamera> camera_;
  cv::Mat frame_;
  size_t length_;
  double timestamp_;
  std::thread thread_;
  std::mutex mutex_;
};

}  // namespace camera
