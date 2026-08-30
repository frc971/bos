#include "src/camera/uvc_camera.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include "absl/status/status.h"
#include "src/utils/pch.h"

namespace camera {

const cv::Mat UVCCamera::backup_image_ =
    cv::imread("/bos/constants/dont_worry_about_it.jpg");

void callback(uvc_frame_t* frame, void* ptr) {
  auto ptr_ = static_cast<UVCCamera*>(ptr);
  CHECK(frame->frame_format == UVC_COLOR_FORMAT_MJPEG);
  auto jpeg_buffer = std::make_unique<JpegBuffer>(
      static_cast<char*>(frame->data), frame->data_bytes);
  jpeg_buffer->timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  std::unique_lock<std::mutex> lock_(ptr_->mutex_, std::try_to_lock);
  if (!lock_.owns_lock()) {
    return;
  }
  ptr_->buffer_ = std::move(jpeg_buffer);
}

UVCCamera::UVCCamera(const CameraConstant& camera_constant,
                     absl::Status& status, std::optional<std::string> log_path)
    : camera_constant_(camera_constant), log_path_(std::move(log_path)) {
  if (!camera_constant.serial_id.has_value()) {
    status = absl::InvalidArgumentError(fmt::format(
        "Must provide a serial id for uvc camera {}", camera_constant.name));
    return;
  }
  int res = uvc_init(&context_, nullptr);
  if (res != 0) {
    status = absl::AbortedError(
        fmt::format("Unable to create context for camera {} with error code {}",
                    camera_constant.name, res));
    return;
  }
  res = uvc_find_device(context_, &device_, 0, 0,
                        camera_constant_.serial_id->c_str());
  if (res != 0) {
    status = absl::AbortedError(
        fmt::format("Unable to find device for camera {} with error code {}",
                    camera_constant.name, res));
    return;
  }
  res = uvc_open(device_, &device_handle_);
  if (res != 0) {
    status = absl::AbortedError(
        fmt::format("Unable to get handle for camera {} with error code {}",
                    camera_constant.name, res));
    return;
  }

  res = uvc_get_stream_ctrl_format_size(
      device_handle_, &ctrl_, UVC_FRAME_FORMAT_MJPEG,
      camera_constant_.frame_width.value(),
      camera_constant_.frame_height.value(), camera_constant_.fps.value());
  if (res != 0) {
    status = absl::AbortedError(fmt::format(
        "Unable to get stream control for camera {} with error code {}",
        camera_constant.name, res));
    return;
  }
  uvc_print_stream_ctrl(&ctrl_, stderr);
  ctrl_.dwMaxPayloadTransferSize =
      camera_constant.max_payload_size.value_or(ctrl_.dwMaxPayloadTransferSize);
  ctrl_.dwMaxVideoFrameSize =
      camera_constant.max_frame_size.value_or(ctrl_.dwMaxVideoFrameSize);
  res = uvc_start_streaming(device_handle_, &ctrl_, callback, this, 0);
  if (res != 0) {
    status = absl::AbortedError(
        fmt::format("Unable to start stream for camera {} with error code {}",
                    camera_constant.name, res));
    return;
  }
}

auto UVCCamera::GetFrame() -> timestamped_frame_t {
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (!buffer_) {
    return {.frame = backup_image_,
            .timestamp = frc::Timer::GetFPGATimestamp().to<double>(),
            .invalid = true};
  }

  timestamped_frame_t timestamped_frame;
  cv::Mat frame = cv::imdecode(buffer_->data, UVCCamera::read_type);
  if (frame.empty()) {
    backup_image_.copyTo(frame);
    timestamped_frame.invalid = true;
  }
  timestamped_frame.frame = frame;
  timestamped_frame.timestamp = buffer_->timestamp;

  return timestamped_frame;
}

auto UVCCamera::Restart() -> void {
  uvc_stop_streaming(device_handle_);
  uvc_close(device_handle_);
  uvc_unref_device(device_);

  uvc_find_device(context_, &device_, 0, 0,
                  camera_constant_.serial_id->c_str());
  uvc_open(device_, &device_handle_);

  LOG(INFO) << "Restarting device UVC Camera. Device ctrl: ";
  uvc_print_stream_ctrl(&ctrl_, stderr);
  LOG(INFO) << "-----------------------------------";
  uvc_start_streaming(device_handle_, &ctrl_, callback, this, 0);
}

UVCCamera::~UVCCamera() {
  uvc_stop_streaming(device_handle_);
  uvc_close(device_handle_);
  uvc_unref_device(device_);
  uvc_exit(context_);
}

auto UVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

}  // namespace camera
