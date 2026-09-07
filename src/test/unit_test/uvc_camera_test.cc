#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

#include "src/camera/simulated_uvc_camera.h"

namespace camera::test {
namespace {

using testing::_;

auto Constants() -> camera_constant_t {
  return {.name = "fake-uvc",
          .frame_width = 1280,
          .frame_height = 800,
          .fps = 30,
          .camera_type = CameraType::UVC};
}

auto LogFolder() -> std::filesystem::path {
  return BOS_SOURCE_DIR "/bos-logs/log181/left";
}

TEST(UvcCameraTest, ReplaysJpegThroughProductionCamera) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  const auto frame = camera.GetFrame();

  EXPECT_FALSE(frame.invalid);
  EXPECT_EQ(frame.frame.size(), cv::Size(1280, 800));
  EXPECT_EQ(frame.frame.channels(), 1);
}

TEST(UvcCameraTest, RestartCallsLibuvcAndContinuesProducingFrames) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  EXPECT_CALL(camera.mock_uvc(), StopStreaming(_)).Times(1);
  EXPECT_CALL(camera.mock_uvc(), Close(_)).Times(1);
  EXPECT_CALL(camera.mock_uvc(), UnrefDevice(_)).Times(1);
  EXPECT_CALL(camera.mock_uvc(), FindDevice()).Times(1);
  EXPECT_CALL(camera.mock_uvc(), Open()).Times(1);
  EXPECT_CALL(camera.mock_uvc(), StartStreaming(_, _)).Times(1);

  camera.Restart();
  EXPECT_FALSE(camera.GetFrame().invalid);
  testing::Mock::VerifyAndClearExpectations(&camera.mock_uvc());
}

TEST(UvcCameraTest, SupportsMultipleIndependentCameras) {
  absl::Status left_status;
  absl::Status right_status;
  SimulatedUvcCamera left(LogFolder(), Constants(), left_status);
  SimulatedUvcCamera right(BOS_SOURCE_DIR "/bos-logs/log181/right", Constants(),
                           right_status);

  ASSERT_TRUE(left_status.ok()) << left_status;
  ASSERT_TRUE(right_status.ok()) << right_status;
  EXPECT_FALSE(left.GetFrame().invalid);
  EXPECT_FALSE(right.GetFrame().invalid);
}

TEST(UvcCameraTest, ResumesAfterReceivingNoFrameForAWhile) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  camera.PauseNextFrame(std::chrono::milliseconds(20));
  const auto start = std::chrono::steady_clock::now();
  const auto frame = camera.GetFrame();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, std::chrono::milliseconds(20));
  EXPECT_FALSE(frame.invalid);
}

TEST(UvcCameraTest, SendsEmptyPayloadThroughProductionCallback) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  EXPECT_THROW(camera.InjectEmptyFrame(), cv::Exception);
}

TEST(UvcCameraTest, RecoversAfterTruncatedJpeg) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  EXPECT_TRUE(camera.InjectCorruptFrame());
  EXPECT_FALSE(camera.GetFrame().invalid);
}

TEST(UvcCameraTest, AutomaticallyDelaysFramesUsingProbabilityTable) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;
  camera.SetFailureProbabilities(
      {.frame_delay = 1.0, .delay = std::chrono::milliseconds(20)}, 1234);

  const auto start = std::chrono::steady_clock::now();
  const auto frame = camera.GetFrame();

  EXPECT_GE(std::chrono::steady_clock::now() - start,
            std::chrono::milliseconds(20));
  EXPECT_FALSE(frame.invalid);
}

TEST(UvcCameraTest, AutomaticallyInjectsEmptyFramesUsingProbabilityTable) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;
  camera.SetFailureProbabilities({.empty = 1.0}, 1234);

  EXPECT_THROW(camera.GetFrame(), cv::Exception);
}

TEST(UvcCameraTest, AutomaticallyInjectsCorruptFramesUsingProbabilityTable) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;
  camera.SetFailureProbabilities({.corrupt = 1.0}, 1234);

  // TODO expect true after making cameras reject corrupt frames
  EXPECT_FALSE(camera.GetFrame().invalid);
  EXPECT_FALSE(camera.GetFrame().invalid);
}

TEST(UvcCameraTest, RejectsInvalidFailureProbabilityTable) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  EXPECT_THROW(camera.SetFailureProbabilities(
                   {.frame_delay = 0.5, .empty = 0.5, .corrupt = 0.5}),
               std::invalid_argument);
  EXPECT_THROW(camera.SetFailureProbabilities({.empty = -0.1}),
               std::invalid_argument);
  EXPECT_THROW(camera.SetFailureProbabilities(
                   {.corrupt = std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);
}

TEST(UvcCameraTest, CallbackDropsFrameRatherThanDeadlockingOnBusyMutex) {
  absl::Status status;
  SimulatedUvcCamera camera(LogFolder(), Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  std::vector<unsigned char> jpeg{0xff, 0xd8, 0xff, 0xd9};
  std::lock_guard lock(camera.production_camera().mutex_);
  EXPECT_TRUE(camera.mock_uvc().DeliverJpeg(jpeg));
}

}  // namespace
}  // namespace camera::test
