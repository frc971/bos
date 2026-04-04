#include "cscore_streamer.h"
namespace camera {

CscoreStreamer::CscoreStreamer(const std::string& name, uint port, uint fps,
                               uint width, uint height, bool verbose)
    : width_(width), height_(height) {
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

CscoreStreamer::CscoreStreamer(const std::string& name, uint port, uint fps,
                               const cv::Mat& frame, float ratio, bool verbose)
    : CscoreStreamer(name, port, fps, frame.cols * ratio, frame.rows * ratio,
                     verbose) {}

void CscoreStreamer::WriteFrame(const cv::Mat& mat) {
  cv::Mat frame;
  cv::resize(mat, frame, cv::Size(width_, height_));
  if (frame.channels() == 4) {
    cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
  }
  source_.PutFrame(frame);
}
}  // namespace camera
