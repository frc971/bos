#pragma once

#include <arpa/inet.h>
#include <cscore_cv.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>
namespace camera {

struct IMX296Streamer {
  inline IMX296Streamer(std::string name, uint port, uint fps = 30)
      : port(port), name(name), fps(fps) {}
  uint width = 1080;
  uint height = 1080;
  uint port;
  std::string name;
  uint fps;
};
// https://stackoverflow.com/questions/5120768/how-to-implement-the-factory-method-pattern-in-c-correctly
// Simple wrapper for cscore streaming to mjpeg
class CscoreStreamer {
 public:
  CscoreStreamer(std::string name, uint port, uint fps, uint width, uint height,
                 bool verbose = false);
  CscoreStreamer(IMX296Streamer streamer);
  void WriteFrame(cv::Mat& mat);

 private:
  cs::CvSource source_;
  cs::MjpegServer server_;
};

}  // namespace camera
