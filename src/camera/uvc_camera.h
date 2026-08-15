#pragma once
#include <memory>
#include "camera_constants.h"
#include "libuvc/libuvc.h"
#include "src/camera/camera.h"
#include "src/camera/uvc_frame_callback.h"
#include "src/utils/pch.h"

namespace camera {

// Wrap opencv's camera into the ICamera interface
class UVCCamera : public ICamera {
 public:
  UVCCamera(const CameraConstant& camera_constant, absl::Status& status,
            std::optional<std::string> log_path = std::nullopt,
            int log_frequency = 0);
  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;
  ~UVCCamera() override;
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;

 private:
  const camera_constant_t camera_constant_;
  std::optional<std::string> log_path_;
  static const cv::Mat backup_image_;
  UVCFrameCallbackReceiver frame_receiver_;
  uvc_context_t* context_ = nullptr;
  uvc_device_t* device_ = nullptr;
  uvc_device_handle_t* device_handle_ = nullptr;
  uvc_stream_ctrl_t ctrl_{};
  std::uint64_t previous_publication_ = 0;
  bool streaming_ = false;
};

}  // namespace camera
