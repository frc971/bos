#pragma once

#include <arpa/inet.h>
#include <cscore_cv.h>
#include "src/utils/pch.h"
namespace camera {

// Wrapper for wpilib's cscore streamer. Streams video to localhost to specified port. Can be accessed remotly with ip.
// eg 10.9.71.101:5801
class CscoreStreamer {
 public:
  CscoreStreamer(const std::string& name, uint port, uint fps, uint width,
                 uint height, bool verbose = false);

  // The resulting size of the streamed image is ratio * frame.width, ration * frame.height
  CscoreStreamer(const std::string& name, uint port, uint fps,
                 const cv::Mat& frame, float ratio = 1.0f,
                 bool verbose = false);

  void WriteFrame(const cv::Mat& mat);

 private:
  uint width_;
  uint height_;
  cs::CvSource source_;
  cs::MjpegServer server_;
};

}  // namespace camera
