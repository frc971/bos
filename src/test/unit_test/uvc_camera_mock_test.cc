#include <cstdlib>
#include <filesystem>
#include <unistd.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "absl/status/status.h"
#include "src/camera/simulated_uvc_camera.h"
#include "src/camera/uvc_camera.h"

namespace {

using ::testing::InSequence;
using ::testing::Return;

class MockUvcHardware {
 public:
  MOCK_METHOD(uvc_error_t, Init, ());
  MOCK_METHOD(uvc_error_t, FindDevice, ());
  MOCK_METHOD(uvc_error_t, Open, ());
  MOCK_METHOD(uvc_error_t, Negotiate, ());
  MOCK_METHOD(uvc_error_t, Start,
              (uvc_frame_callback_t * callback, void* user));
  MOCK_METHOD(void, Stop, ());
  MOCK_METHOD(void, Close, ());
  MOCK_METHOD(void, Unref, ());
  MOCK_METHOD(void, Exit, ());

  uvc_frame_callback_t* callback = nullptr;
  void* callback_user = nullptr;
};

MockUvcHardware* hardware = nullptr;

auto* const kContext = reinterpret_cast<uvc_context_t*>(0x101);
auto* const kDevice = reinterpret_cast<uvc_device_t*>(0x102);
auto* const kHandle = reinterpret_cast<uvc_device_handle_t*>(0x103);

auto Constants() -> camera::camera_constant_t {
  return {.name = "mock-uvc",
          .frame_width = 1280,
          .frame_height = 800,
          .fps = 30,
          .max_frame_size = 2U * 1024U * 1024U,
          .max_payload_size = 2U * 1024U * 1024U};
}

class UVCCameraMockTest : public ::testing::Test {
 protected:
  void SetUp() override { hardware = &mock_; }
  void TearDown() override { hardware = nullptr; }

  void ExpectConstruction() {
    EXPECT_CALL(mock_, Init).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, FindDevice).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, Open).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, Negotiate).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, Start)
        .WillOnce([this](uvc_frame_callback_t* callback, void* user) {
          mock_.callback = callback;
          mock_.callback_user = user;
          return UVC_SUCCESS;
        });
  }

  void ExpectDestruction() {
    EXPECT_CALL(mock_, Stop);
    EXPECT_CALL(mock_, Close);
    EXPECT_CALL(mock_, Unref);
    EXPECT_CALL(mock_, Exit);
  }

  MockUvcHardware mock_;
};

TEST_F(UVCCameraMockTest, EmptyFrameDoesNotKeepCallbackMutexLocked) {
  ExpectConstruction();
  ExpectDestruction();
  absl::Status status;
  camera::UVCCamera camera(Constants(), status);
  ASSERT_TRUE(status.ok()) << status;
  ASSERT_NE(mock_.callback, nullptr);

  camera.frame_index_ = 0;
  camera.previous_frame_index_ = 0;
  unsigned char truncated_jpeg[] = {0xff, 0xd8, 0xff};
  uvc_frame_t empty{};
  empty.data = truncated_jpeg;
  empty.data_bytes = sizeof(truncated_jpeg);
  empty.frame_format = UVC_COLOR_FORMAT_MJPEG;
  empty.sequence = 1;
  mock_.callback(&empty, mock_.callback_user);

  ASSERT_TRUE(camera.mutex_.try_lock())
      << "empty-frame callback leaked the UVCCamera mutex";
  camera.mutex_.unlock();
}

TEST_F(UVCCameraMockTest, FrameFaultsReturnWithoutLeakingMutex) {
  ExpectConstruction();
  ExpectDestruction();
  absl::Status status;
  camera::UVCCamera camera(Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  unsigned char corrupt_jpeg[] = {0xff, 0xd8, 0xff};
  const auto unsupported =
      static_cast<decltype(uvc_frame_t{}.frame_format)>(255);
  for (const auto format : {UVC_COLOR_FORMAT_MJPEG, unsupported}) {
    uvc_frame_t fault{};
    fault.data = corrupt_jpeg;
    fault.data_bytes = sizeof(corrupt_jpeg);
    fault.frame_format = format;
    fault.sequence = 1;
    mock_.callback(&fault, mock_.callback_user);
    ASSERT_TRUE(camera.mutex_.try_lock());
    camera.mutex_.unlock();
  }
}

TEST_F(UVCCameraMockTest, StreamingInitializationFailureIsReported) {
  EXPECT_CALL(mock_, Init).WillOnce(Return(UVC_SUCCESS));
  EXPECT_CALL(mock_, FindDevice).WillOnce(Return(UVC_SUCCESS));
  EXPECT_CALL(mock_, Open).WillOnce(Return(UVC_SUCCESS));
  EXPECT_CALL(mock_, Negotiate).WillOnce(Return(UVC_SUCCESS));
  EXPECT_CALL(mock_, Start).WillOnce(Return(UVC_ERROR_IO));
  ExpectDestruction();

  absl::Status status;
  camera::UVCCamera camera(Constants(), status);
  EXPECT_FALSE(status.ok());
}

TEST_F(UVCCameraMockTest, CallbackNeverBlocksOnContendedMutex) {
  ExpectConstruction();
  ExpectDestruction();
  absl::Status status;
  camera::UVCCamera camera(Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  uvc_frame_t empty{};
  unsigned char truncated_jpeg[] = {0xff, 0xd8, 0xff};
  empty.data = truncated_jpeg;
  empty.data_bytes = sizeof(truncated_jpeg);
  empty.frame_format = UVC_COLOR_FORMAT_MJPEG;
  empty.sequence = 1;
  // Run the deliberately contended callback in a subprocess. If try_lock is
  // changed to lock, SIGALRM terminates the child and this assertion fails
  // instead of wedging the complete test run.
  EXPECT_EXIT(
      {
        alarm(1);
        camera.mutex_.lock();
        mock_.callback(&empty, mock_.callback_user);
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(UVCCameraMockTest, RestartReconstructsHardwareAndAcceptsDiskFrame) {
  ExpectConstruction();
  absl::Status status;
  camera::UVCCamera uvc_camera(Constants(), status);
  ASSERT_TRUE(status.ok()) << status;

  {
    InSequence sequence;
    EXPECT_CALL(mock_, Stop);
    EXPECT_CALL(mock_, Close);
    EXPECT_CALL(mock_, Unref);
    EXPECT_CALL(mock_, FindDevice).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, Open).WillOnce(Return(UVC_SUCCESS));
    EXPECT_CALL(mock_, Start)
        .WillOnce([this](uvc_frame_callback_t* callback, void* user) {
          mock_.callback = callback;
          mock_.callback_user = user;
          return UVC_SUCCESS;
        });
  }
  uvc_camera.Restart();
  ExpectDestruction();

  const char* configured_folder = std::getenv("BOS_UVC_TEST_IMAGE_DIR");
  const std::filesystem::path folder =
      configured_folder != nullptr
          ? configured_folder
          : BOS_SOURCE_DIR "/bos_logs/log181/right";
  camera::SimulatedUVCFrameSource frames(folder, status);
  ASSERT_TRUE(status.ok()) << status;
  const auto expected_frames = frames.size();
  uvc_camera.frame_index_ = -1;
  uvc_camera.previous_frame_index_ = -1;
  frames.DeliverNext(mock_.callback, mock_.callback_user,
                     camera::SimulatedUVCFrameFault::kEmpty);
  ASSERT_TRUE(uvc_camera.mutex_.try_lock());
  uvc_camera.mutex_.unlock();
  frames.Rewind();
  std::size_t decoded_frames = 0;
  while (!frames.empty()) {
    frames.DeliverNext(mock_.callback, mock_.callback_user);
    const auto decoded = uvc_camera.GetFrame();
    ASSERT_FALSE(decoded.invalid);
    ASSERT_FALSE(decoded.frame.empty());
    ++decoded_frames;
  }
  EXPECT_EQ(decoded_frames, expected_frames);
}

}  // namespace

extern "C" {

uvc_error_t uvc_init(uvc_context_t** context, struct libusb_context*) {
  const auto result = hardware->Init();
  if (result == UVC_SUCCESS) *context = kContext;
  return result;
}
void uvc_exit(uvc_context_t*) { hardware->Exit(); }
uvc_error_t uvc_find_device(uvc_context_t*, uvc_device_t** device, int, int,
                            const char*) {
  const auto result = hardware->FindDevice();
  if (result == UVC_SUCCESS) *device = kDevice;
  return result;
}
uvc_error_t uvc_open(uvc_device_t*, uvc_device_handle_t** handle) {
  const auto result = hardware->Open();
  if (result == UVC_SUCCESS) *handle = kHandle;
  return result;
}
void uvc_close(uvc_device_handle_t*) { hardware->Close(); }
void uvc_unref_device(uvc_device_t*) { hardware->Unref(); }
uvc_error_t uvc_get_stream_ctrl_format_size(
    uvc_device_handle_t*, uvc_stream_ctrl_t* control, enum uvc_frame_format,
    int, int, int) {
  const auto result = hardware->Negotiate();
  if (result == UVC_SUCCESS) *control = {};
  return result;
}
void uvc_print_stream_ctrl(uvc_stream_ctrl_t*, FILE*) {}
uvc_error_t uvc_start_streaming(uvc_device_handle_t*, uvc_stream_ctrl_t*,
                                uvc_frame_callback_t* callback, void* user,
                                uint8_t) {
  return hardware->Start(callback, user);
}
void uvc_stop_streaming(uvc_device_handle_t*) { hardware->Stop(); }
uvc_frame_t* uvc_allocate_frame(size_t) { return nullptr; }
void uvc_free_frame(uvc_frame_t*) {}
uvc_error_t uvc_yuyv2bgr(uvc_frame_t*, uvc_frame_t*) {
  return UVC_ERROR_NOT_SUPPORTED;
}

}  // extern "C"
