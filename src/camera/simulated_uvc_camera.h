#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include <gmock/gmock.h>

#include "absl/status/status.h"
#include "libuvc/libuvc.h"
#include "src/camera/uvc_camera.h"

namespace camera::test {

class MockUvcApi {
 public:
  MockUvcApi();

  MOCK_METHOD(uvc_error_t, Init, ());
  MOCK_METHOD(void, Exit, (uvc_context_t*));
  MOCK_METHOD(uvc_error_t, FindDevice, ());
  MOCK_METHOD(uvc_error_t, Open, ());
  MOCK_METHOD(void, Close, (uvc_device_handle_t*));
  MOCK_METHOD(void, UnrefDevice, (uvc_device_t*));
  MOCK_METHOD(uvc_error_t, GetStreamControl,
              (uvc_stream_ctrl_t*, int width, int height, int fps));
  MOCK_METHOD(uvc_error_t, StartStreaming,
              (uvc_frame_callback_t*, void* user));
  MOCK_METHOD(void, StopStreaming, (uvc_device_handle_t*));

  auto DeliverJpeg(std::vector<unsigned char>& bytes) -> bool;
  auto DeliverEmptyFrame() -> bool;

 private:
  auto Deliver(void* data, std::size_t size) -> bool;

  uvc_frame_callback_t* callback_ = nullptr;
  void* callback_user_ = nullptr;
  uint32_t sequence_ = 0;
};

class SimulatedUvcCamera final : public ICamera {
 public:
  SimulatedUvcCamera(const std::filesystem::path& image_folder,
                     const camera_constant_t& constants, absl::Status& status);
  ~SimulatedUvcCamera() override;

  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;
  [[nodiscard]] auto IsDone() -> bool override;

  auto PauseNextFrame(std::chrono::milliseconds delay) -> void;
  auto InjectEmptyFrame() -> bool;
  auto InjectCorruptFrame() -> bool;

  auto mock_uvc() -> MockUvcApi& { return mock_uvc_; }
  auto production_camera() -> UVCCamera& { return *camera_; }

 private:
  auto ReadNextJpeg() -> std::vector<unsigned char>;

  std::vector<std::filesystem::path> image_paths_;
  std::size_t next_image_ = 0;
  std::optional<std::chrono::milliseconds> next_frame_delay_;
  testing::NiceMock<MockUvcApi> mock_uvc_;
  std::unique_ptr<UVCCamera> camera_;
};

}  // namespace camera::test
