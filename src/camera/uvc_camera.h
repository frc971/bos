#pragma once
#include <memory>
#include "camera_constants.h"
#include "libuvc/libuvc.h"
#include "src/camera/camera.h"
#include "src/utils/pch.h"

namespace camera {

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

 private:
  auto StartCamera(uvc_stream_ctrl_t ctrl) -> void;
};

}  // namespace camera
