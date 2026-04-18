#pragma once
#include <chrono>
#include <queue>
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/utils/pch.h"
#include "src/utils/timer.h"

namespace camera {

using timestamped_frame_path_t = struct TimestampedFramePath {
  std::string path;
  double timestamp;
};

struct CompareTimestampedFramePath {
  auto operator()(const timestamped_frame_path_t& a,
                  const timestamped_frame_path_t& b) -> bool {
    return a.timestamp > b.timestamp;
  }
};

// Replays log of camera feed into ICamera. Frames are "captured" at the same time that the real camera capptured them according to wpilib GetFGPATimestamp() time.
// Can be used to testing
class DiskCamera : public ICamera {
 public:
  DiskCamera(std::string image_folder_path,
             std::optional<camera_constant_t> camera_constant = std::nullopt,
             double speed = 1.0, std::optional<double> start = std::nullopt,
             std::optional<double> end = std::nullopt);
  void GetFrame(timestamped_frame_t* output) override;
  auto Restart() -> void override;
  auto IsDone() -> bool override { return image_paths_.empty(); }
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;

 private:
  const double speed_;
  const std::optional<double> start_, end_;
  std::optional<camera_constant_t> camera_constant_;
  std::string image_folder_path_;
  std::priority_queue<TimestampedFramePath, std::vector<TimestampedFramePath>,
                      CompareTimestampedFramePath>
      image_paths_;
};

}  // namespace camera
