#include "cscore_streamer.h"
namespace camera {

auto NormalizeForStreaming(const cv::Mat& mat) -> cv::Mat {
  if (mat.empty()) {
    return {};
  }

  switch (mat.channels()) {
    case 1: {
      cv::Mat bgr;
      cv::cvtColor(mat, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }
    case 2: {
      cv::Mat bgr;
      cv::cvtColor(mat, bgr, cv::COLOR_YUV2BGR_YUY2);
      return bgr;
    }
    case 3:
      return mat;
    case 4: {
      cv::Mat bgr;
      cv::cvtColor(mat, bgr, cv::COLOR_BGRA2BGR);
      return bgr;
    }
    default:
      std::cerr << "Unsupported frame channel count for CSCore: "
                << mat.channels();
      return {};
  }
}

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
  std::cout << "WRITING FRAME";
  cv::Mat normalized = NormalizeForStreaming(mat);
  if (normalized.empty()) {
    return;
  }

  cv::Mat frame;
  cv::resize(normalized, frame, cv::Size(width_, height_));
  source_.PutFrame(frame);
  std::cout << "WROTE FRAME";
  std::this_thread::sleep_for(std::chrono::duration<double>(1));
  std::cout << "End sleep";
}
}  // namespace camera
