#pragma once

#include <arpa/inet.h>
#include <cscore_cv.h>
#include "src/utils/pch.h"
namespace camera {

// Wrapper for wpilib's cscore streamer. Streams video to localhost to specified port. Can be accessed remotly with ip.
// eg 10.9.71.101:4971
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
