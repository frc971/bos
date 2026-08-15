#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"

namespace camera {

struct SimulatedUVCInitializationFaults {
  double context = 0.0;
  double discovery = 0.0;
  double open = 0.0;
  double negotiation = 0.0;
  double streaming_start = 0.0;
};

struct SimulatedUVCFrameFaults {
  double timeout = 0.0;
  double temporary_stall = 0.0;
  double permanent_stall = 0.0;
  double overflow = 0.0;
  double transfer_error = 0.0;
  double no_device = 0.0;
  double corruption = 0.0;
  double empty_frame = 0.0;
  double oversized_frame = 0.0;
  double unsupported_format = 0.0;
  double fatal_abort = 0.0;
};

struct SimulatedUVCDurationRange {
  double min_seconds = 0.0;
  double max_seconds = 0.0;
};

struct SimulatedUVCCameraConfig {
  std::filesystem::path image_directory;
  double replay_speed = 1.0;
  std::optional<double> start;
  std::optional<double> end;
  std::optional<std::uint64_t> random_seed;
  SimulatedUVCInitializationFaults initialization_faults;
  SimulatedUVCFrameFaults frame_faults;
  SimulatedUVCDurationRange temporary_stall;
  SimulatedUVCDurationRange processing_delay;
};

struct SimulatedUVCCameraStatistics {
  std::uint64_t source_frames = 0;
  std::uint64_t transfers = 0;
  std::uint64_t retries = 0;
  std::uint64_t retired_buffers = 0;
  std::uint64_t assembled_frames = 0;
  std::uint64_t hold_buffer_overwrites = 0;
  std::uint64_t callback_drops = 0;
  std::uint64_t decode_failures = 0;
  std::uint64_t sequence_gaps = 0;
  std::uint64_t deliveries = 0;
  std::uint64_t injected_faults = 0;
  std::uint64_t eof = 0;
  std::uint64_t restarts = 0;
};

// A deterministic, disk-backed simulation of the libuvc/BOS MJPEG pipeline.
class SimulatedUVCCamera final : public ICamera {
 public:
  SimulatedUVCCamera(const camera_constant_t& camera_constant,
                     SimulatedUVCCameraConfig config, absl::Status& status);
  ~SimulatedUVCCamera() override;

  SimulatedUVCCamera(const SimulatedUVCCamera&) = delete;
  auto operator=(const SimulatedUVCCamera&) -> SimulatedUVCCamera& = delete;

  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;
  auto IsDone() -> bool override;
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;
  [[nodiscard]] auto GetStatistics() const -> SimulatedUVCCameraStatistics;

 private:
  class State;
  const camera_constant_t camera_constant_;
  const SimulatedUVCCameraConfig config_;
  std::unique_ptr<State> state_;
};

}  // namespace camera
