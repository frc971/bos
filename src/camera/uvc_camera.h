#pragma once
#include <memory>
#include "camera_constants.h"
#include "libuvc/libuvc.h"
#include "src/camera/camera.h"
#include "src/utils/pch.h"

namespace camera {

struct JpegBuffer {
  JpegBuffer(char* ptr, size_t size_) : data(ptr, ptr + size_), size(size_) {}
  std::vector<char> data;
  size_t size;
  double timestamp = 0;
};

// Wrap opencv's camera into the ICamera interface
class UVCCamera : public ICamera {
 public:
  UVCCamera(const CameraConstant& camera_constant, absl::Status& status,
            std::optional<std::string> log_path = std::nullopt);
  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;
  ~UVCCamera() override;
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;

 public:
  const camera_constant_t camera_constant_;
  std::optional<std::string> log_path_;
  static const cv::Mat backup_image_;
  uvc_context_t* context_;
  uvc_device_t* device_;
  uvc_device_handle_t* device_handle_;
  timestamped_frame_t frame_buffer;
  uvc_stream_ctrl_t ctrl_;
  std::mutex mutex_;
  int frame_index_;
  int previous_frame_index_;
  std::unique_ptr<JpegBuffer> buffer_;
  static constexpr cv::ImreadModes read_type = cv::IMREAD_GRAYSCALE;

 private:
  auto StartCamera(uvc_stream_ctrl_t ctrl) -> void;
};

}  // namespace camera
