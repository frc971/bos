#pragma once
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <queue>
#include <string>
#include "src/camera/camera.h"
#include "src/camera/camera_source.h"

namespace camera {

using timestamped_frame_path_t = struct TimestampedFramePath {
  std::string path;
  double timestamp;
};

struct CompareTimestampedFramePath {
  auto operator()(const timestamped_frame_path_t& a,
                  const timestamped_frame_path_t& b) -> bool {
    return a.timestamp < b.timestamp;
  }
};

class DiskCamera : public ICamera {
 public:
  DiskCamera(std::string image_folder_path);
  void GetFrame(cv::Mat& frame) override;

 private:
  std::string image_folder_path_;
  std::priority_queue<TimestampedFramePath, std::vector<TimestampedFramePath>,
                      CompareTimestampedFramePath>
      image_paths_;
  int current_frame_;
};

}  // namespace camera
