#include "src/camera/uvc_camera.h"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "absl/status/status.h"
#include "src/utils/pch.h"

namespace camera {

const cv::Mat UVCCamera::backup_image_ =
    cv::imread("/bos/constants/dont_worry_about_it.jpg");

UVCCamera::UVCCamera(const CameraConstant& camera_constant,
                     absl::Status& status, std::optional<std::string> log_path,
                     int log_frequency)
    : camera_constant_(camera_constant),
      log_path_(std::move(log_path)),
      frame_receiver_(camera_constant_, log_path_, log_frequency) {

  if (log_path_.has_value()) {
    std::error_code error;
    std::filesystem::create_directories(*log_path_, error);
    if (error) {
      LOG(WARNING) << "Failed to create camera log folder " << *log_path_
                   << ": " << error.message();
    }
  }

  int res = uvc_init(&context_, nullptr);
  if (res != 0) {
    status = absl::AbortedError(
        fmt::format("Unable to create context for camera {} with error code {}",
                    camera_constant.name, res));
    return;
  }
  if (!camera_constant.serial_id.has_value()) {
    LOG(WARNING) << "Was not provided with serial id. This shuold only be done "
                    "if there is only one uvc camera connected";

    res = uvc_find_device(context_, &device_, 0, 0, nullptr);
  } else {
    res = uvc_find_device(context_, &device_, 0, 0,
                          camera_constant_.serial_id->c_str());
    LOG(INFO) << "Serial id: " << camera_constant_.serial_id.value();
  }
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
    status = absl::AbortedError("Unable to get stream format for camera: " +
                                camera_constant.name);
    return;
  }
  uvc_print_stream_ctrl(&ctrl_, stderr);
  ctrl_.dwMaxPayloadTransferSize =
      camera_constant.max_payload_size.value_or(ctrl_.dwMaxPayloadTransferSize);
  ctrl_.dwMaxVideoFrameSize =
      camera_constant.max_frame_size.value_or(ctrl_.dwMaxVideoFrameSize);
  res = uvc_start_streaming(device_handle_, &ctrl_, UVCFrameCallback,
                            &frame_receiver_, 0);
  if (res != 0) {
    status = absl::AbortedError("Unable to start streaming for camera: " +
                                camera_constant.name);
    return;
  }
  streaming_ = true;
}

auto UVCCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t copied_timestamped_frame;
  frame_receiver_.WaitForFrameAfter(previous_publication_);
  if (!frame_receiver_.CopyLatest(&copied_timestamped_frame,
                                  &previous_publication_)) {
    backup_image_.copyTo(copied_timestamped_frame.frame);
    copied_timestamped_frame.invalid = true;
    copied_timestamped_frame.timestamp =
        frc::Timer::GetFPGATimestamp().to<double>();
  }
  return copied_timestamped_frame;
}

auto UVCCamera::Restart() -> void {
  if (streaming_) {
    uvc_stop_streaming(device_handle_);
    streaming_ = false;
  }
  if (device_handle_ != nullptr) {
    uvc_close(device_handle_);
    device_handle_ = nullptr;
  }
  if (device_ != nullptr) {
    uvc_unref_device(device_);
    device_ = nullptr;
  }

  const char* serial_id = camera_constant_.serial_id.has_value()
                              ? camera_constant_.serial_id.value().c_str()
                              : nullptr;
  if (context_ == nullptr ||
      uvc_find_device(context_, &device_, 0, 0, serial_id) != UVC_SUCCESS) {
    LOG(WARNING) << "Unable to find camera " << camera_constant_.name
                 << " while restarting";
    return;
  }
  if (uvc_open(device_, &device_handle_) != UVC_SUCCESS) {
    LOG(WARNING) << "Unable to open camera " << camera_constant_.name
                 << " while restarting";
    return;
  }

  LOG(INFO) << "Restarting device UVC Camera. Device ctrl: ";
  uvc_print_stream_ctrl(&ctrl_, stderr);
  LOG(INFO) << "-----------------------------------";
  if (uvc_start_streaming(device_handle_, &ctrl_, UVCFrameCallback,
                          &frame_receiver_, 0) == UVC_SUCCESS) {
    streaming_ = true;
  }
}

UVCCamera::~UVCCamera() {
  if (streaming_)
    uvc_stop_streaming(device_handle_);
  if (device_handle_ != nullptr)
    uvc_close(device_handle_);
  if (device_ != nullptr)
    uvc_unref_device(device_);
  if (context_ != nullptr)
    uvc_exit(context_);
  LOG(INFO) << camera_constant_.name << " has been destructed";
}

auto UVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

}  // namespace camera
