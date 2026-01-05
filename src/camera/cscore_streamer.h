#pragma once

#include <arpa/inet.h>
#include <cscore_cv.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>
namespace camera {

class CscoreStreamer {
 public:
  CscoreStreamer(const std::string& name, uint port, uint fps, uint width,
                 uint height, bool verbose = false);
  void WriteFrame(cv::Mat& mat);

 private:
  cs::CvSource source_;
  cs::MjpegServer server_;
};

}  // namespace camera
