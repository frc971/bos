#pragma once
#include <queue>
#include "src/camera/camera.h"
#include "src/camera/camera_source.h"
#include "src/utils/pch.h"

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

// Replays log of camera feed into ICamera. Frames are "captured" at the same time that the real camera capptured them according to wpilib GetFGPATimestamp() time.
// Can be used to testing
class DiskCamera : public ICamera {
 public:
  DiskCamera(std::string image_folder_path);
  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;

 private:
  std::string image_folder_path_;
  std::priority_queue<TimestampedFramePath, std::vector<TimestampedFramePath>,
                      CompareTimestampedFramePath>
      image_paths_;
};

}  // namespace camera
