#include "pch.h"

#include "cscore_streamer.h"
#include <cscore_cpp.h>
#include <cscore_cv.h>

namespace camera {

CscoreStreamer::CscoreStreamer(std::string name, uint port, uint fps,
                               uint width, uint height, bool verbose) {
  if (verbose) {
    std::cout << "Initializing CscoreStreamer with the following parameters:"
              << std::endl;
    std::cout << "Name: " << name << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;
  }
  cs::VideoMode mode(cs::VideoMode::kBGR, width, height, fps);
  source_ = cs::CvSource(name, mode);
  server_ = cs::MjpegServer(name, port);
  server_.SetSource(source_);
}

// is this tuff? (prob not)
CscoreStreamer::CscoreStreamer(IMX296Streamer streamer)
    : CscoreStreamer(streamer.name, streamer.port, streamer.fps,
                     streamer.height, streamer.width) {}

void CscoreStreamer::WriteFrame(cv::Mat& mat) {
  source_.PutFrame(mat);
}
}  // namespace camera