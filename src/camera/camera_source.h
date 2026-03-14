#pragma once
#include "src/camera/camera.h"
#include "src/utils/pch.h"
namespace camera {

// Takes in a ICamera and starts a thread to retreive and process the frame from that ICamera and stores it inside timestamped_frame_.
// When a frame is retreived, a cv::Mat with reference to the same underlying memory is created. And the memory should be automaticly deallocated when the reference counter reaches 0. (The reference counter decrements each time a cv::Mat gets destructed)
// cv::Mat's reference counter seems to be atomic, so it behaves similarly to a std::shared_ptr and it is safe to access the same cv::Mat across multiple threads as longs as we do not write to it
class CameraSource {
 public:
  CameraSource(std::string name, std::unique_ptr<ICamera> camera);
  [[nodiscard]] auto Get(bool sim = false) -> timestamped_frame_t;
  [[nodiscard]] auto GetFrame() -> cv::Mat;
  [[nodiscard]] auto GetName() const -> std::string { return name_; }

 private:
  std::string name_;
  std::unique_ptr<ICamera> camera_;
  timestamped_frame_t timestamped_frame_;
  std::thread thread_;
  std::mutex mutex_;
};

}  // namespace camera
