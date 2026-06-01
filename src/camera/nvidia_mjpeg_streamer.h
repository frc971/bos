#pragma once

#include <nadjieb/mjpeg_streamer.hpp>
#include <opencv2/core/mat.hpp>
#include "NvJpegEncoder.h"

namespace camera {

class NvidiaMjpegStreamer {
 public:
  NvidiaMjpegStreamer(uint port);

  void WriteFrame(const cv::Mat& mat);

 private:
  nadjieb::MJPEGStreamer streamer_;
  NvJPEGEncoder* encoder_ = nullptr;
};

}  // namespace camera
