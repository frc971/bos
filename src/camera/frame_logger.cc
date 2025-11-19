#include "frame_logger.h"
#include <filesystem>
#include <opencv2/imgcodecs.hpp>

namespace camera {
FrameLogger::FrameLogger(std::string folder_path)
    : folder_path_(folder_path), frame_index_(0) {}

void FrameLogger::WriteFrame(cv::Mat& frame) {
  std::ostringstream filename;
  filename << folder_path_ << "/" << std::setfill('0') << std::setw(4)
           << frame_index_ << ".png";
  cv::imwrite(filename.str(), frame, {cv::IMWRITE_PNG_COMPRESSION, 0});
  frame_index_++;
}

}  // namespace camera
