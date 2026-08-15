#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

#include <gtest/gtest.h>

#include "absl/status/status.h"
#include "src/camera/simulated_uvc_camera.h"

namespace {

constexpr char kLogFolder[] = BOS_SOURCE_DIR "/bos_logs/log181/right";
constexpr std::uint64_t kSeed = 0x5eed1234ULL;

auto CameraConstants() -> camera::camera_constant_t {
  return {.name = "simulated-test",
          .frame_width = 1280,
          .frame_height = 800,
          .fps = 30,
          .max_frame_size = 2U * 1024U * 1024U,
          .max_payload_size = 2U * 1024U * 1024U};
}

auto Config(bool one_frame = false) -> camera::SimulatedUVCCameraConfig {
  return {.image_directory = kLogFolder,
          .replay_speed = 1.0e9,
          .end = one_frame ? std::optional<double>(0.0) : std::nullopt,
          .random_seed = kSeed};
}

void WaitUntilDone(camera::SimulatedUVCCamera& camera) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!camera.IsDone() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(camera.IsDone()) << "simulator did not drain";
}

auto NumericFrameCount() -> std::uint64_t {
  std::uint64_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(kLogFolder)) {
    if (!entry.is_regular_file())
      continue;
    const std::string stem = entry.path().stem().string();
    std::size_t consumed = 0;
    try {
      (void)std::stod(stem, &consumed);
    } catch (const std::exception&) {
      continue;
    }
    count += consumed == stem.size();
  }
  return count;
}

TEST(SimulatedUVCCameraTest, FixedSeedFaultFrequencyAcrossFullLog) {  // NOLINT
  auto config = Config();
  constexpr double kProbability = 0.25;
  config.frame_faults.corruption = kProbability;

  absl::Status status;
  camera::SimulatedUVCCamera simulated(CameraConstants(), std::move(config),
                                       status);
  ASSERT_TRUE(status.ok()) << status;
  WaitUntilDone(simulated);

  const auto statistics = simulated.GetStatistics();
  const std::uint64_t frame_count = NumericFrameCount();
  ASSERT_EQ(statistics.source_frames, frame_count);
  ASSERT_EQ(statistics.eof, 1U);

  const double expected = kProbability * static_cast<double>(frame_count);
  const double sigma = std::sqrt(expected * (1.0 - kProbability));
  EXPECT_NEAR(static_cast<double>(statistics.injected_faults), expected,
              4.0 * sigma);
}

TEST(SimulatedUVCCameraTest, InjectedFaultsExposeExpectedSymptoms) {  // NOLINT
  for (const auto fault : {&camera::SimulatedUVCFrameFaults::empty_frame,
                           &camera::SimulatedUVCFrameFaults::unsupported_format,
                           &camera::SimulatedUVCFrameFaults::corruption}) {
    auto config = Config(true);
    config.frame_faults.*fault = 1.0;
    absl::Status status;
    camera::SimulatedUVCCamera simulated(CameraConstants(), std::move(config),
                                         status);
    ASSERT_TRUE(status.ok()) << status;
    WaitUntilDone(simulated);
    EXPECT_TRUE(simulated.GetFrame().invalid);
    EXPECT_EQ(simulated.GetStatistics().decode_failures, 1U);
  }
  {
    auto clean_config = Config(true);
    absl::Status clean_status;
    camera::SimulatedUVCCamera clean(CameraConstants(), std::move(clean_config),
                                     clean_status);
    ASSERT_TRUE(clean_status.ok()) << clean_status;
    WaitUntilDone(clean);
    const auto clean_frame = clean.GetFrame();

    auto corrupt_config = Config(true);
    corrupt_config.frame_faults.corruption = 1.0;
    absl::Status corrupt_status;
    camera::SimulatedUVCCamera corrupt(
        CameraConstants(), std::move(corrupt_config), corrupt_status);
    ASSERT_TRUE(corrupt_status.ok()) << corrupt_status;
    WaitUntilDone(corrupt);
    const auto corrupt_frame = corrupt.GetFrame();

    ASSERT_FALSE(clean_frame.invalid);
    ASSERT_TRUE(corrupt_frame.invalid);
    ASSERT_TRUE(corrupt_frame.frame.empty());
    EXPECT_EQ(corrupt.GetStatistics().decode_failures, 1U);
    EXPECT_EQ(corrupt.GetStatistics().injected_faults, 1U);
  }
  {
    auto config = Config(true);
    config.frame_faults.overflow = 1.0;
    absl::Status status;
    camera::SimulatedUVCCamera simulated(CameraConstants(), std::move(config),
                                         status);
    ASSERT_TRUE(status.ok()) << status;
    WaitUntilDone(simulated);
    EXPECT_TRUE(simulated.GetFrame().invalid);
    const auto statistics = simulated.GetStatistics();
    EXPECT_EQ(statistics.retired_buffers, 1U);
    EXPECT_EQ(statistics.assembled_frames, 0U);
  }
}

}  // namespace
