#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "libuvc/libuvc.h"
#include "src/camera/camera.h"

namespace camera {

struct UVCFrameCallbackStatistics {
  std::uint64_t attempts = 0;
  std::uint64_t successes = 0;
  std::uint64_t decode_failures = 0;
  std::uint64_t drops = 0;
};

// The common endpoint used by both libuvc and the simulated UVC transport.
// Producers submit uvc_frame_t objects through UVCFrameCallback; this class
// owns all decoding and publication behavior.
class UVCFrameCallbackReceiver {
 public:
  explicit UVCFrameCallbackReceiver(
      const camera_constant_t& camera_constant,
      std::optional<std::string> log_path = std::nullopt,
      int log_frequency = 0)
      : camera_constant_(camera_constant),
        log_path_(std::move(log_path)),
        log_frequency_(log_frequency) {}

  void WaitForFrameAfter(std::uint64_t publication);
  [[nodiscard]] auto CopyLatest(timestamped_frame_t* frame,
                                std::uint64_t* publication = nullptr) -> bool;
  [[nodiscard]] auto Publication() const -> std::uint64_t;
  [[nodiscard]] auto Statistics() const -> UVCFrameCallbackStatistics;

 private:
  friend void UVCFrameCallback(uvc_frame_t* frame, void* receiver);
  void LogCompressedFrame(uvc_frame_t* frame) const;
  void HandleFrame(uvc_frame_t* frame);

  const camera_constant_t& camera_constant_;
  std::optional<std::string> log_path_;
  int log_frequency_;
  mutable std::mutex mutex_;
  std::condition_variable frame_available_;
  timestamped_frame_t frame_;
  std::atomic<std::uint64_t> attempts_{0};
  std::atomic<std::uint64_t> successes_{0};
  std::atomic<std::uint64_t> decode_failures_{0};
  std::atomic<std::uint64_t> drops_{0};
};

// This is deliberately the one callback used for real and simulated frames.
void UVCFrameCallback(uvc_frame_t* frame, void* receiver);

}  // namespace camera
