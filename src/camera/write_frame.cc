#include "write_frame.h"
#include <opencv2/imgcodecs.hpp>

static std::vector<int> compression_params{cv::IMWRITE_PNG_COMPRESSION, 0};

namespace camera {
auto WriteFrame(const std::string& folder,
                timestamped_frame_t& timestamped_frame) -> bool {
  return cv::imwrite(
      folder + "/" + std::to_string(timestamped_frame.timestamp) + ".png",
      timestamped_frame.frame, compression_params);
}

}  // namespace camera
