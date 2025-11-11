#pragma once
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"

namespace camera {

class FrameLogger {
 public:
  FrameLogger(std::string folder_path);
  void WriteFrame(cv::Mat& frame);

 private:
  std::string folder_path_;
  int frame_index_;
};

}  // namespace camera
