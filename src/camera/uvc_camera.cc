#include "src/camera/uvc_camera.h"
#include "absl/status/status.h"

namespace camera {

auto CopyPlane(const NvBuffer::NvBufferPlane& plane, int rows, int cols,
               unsigned char* dst) -> void {

  for (int y = 0; y < rows; ++y) {
    std::memcpy(dst + (y * cols), plane.data + (y * plane.fmt.stride), cols);
  }
}

void callback(uvc_frame_t* frame, void* ptr) {
  // return;
  auto ptr_ = static_cast<UVCCamera*>(ptr);
  if (ptr_->mutex_.try_lock()) {
    switch (frame->frame_format) {
      case UVC_COLOR_FORMAT_MJPEG: {
        auto* data = static_cast<unsigned char*>(frame->data);
        NvBuffer* decoded_buffer = nullptr;
        uint32_t pixfmt = 0;
        uint32_t width = 0;
        uint32_t height = 0;

        const int decode_status = ptr_->decoder_->decodeToBuffer(
            &decoded_buffer, data, frame->data_bytes, &pixfmt, &width, &height);

        if (decode_status < 0 || !decoded_buffer) {
          LOG(WARNING) << "Failed to decode buffer";
          break;
        }

        cv::Mat i420(height + (height / 2), width, CV_8UC1);

        auto* y_dst = i420.ptr<unsigned char>(0);
        auto* u_dst = i420.ptr<unsigned char>(height);
        auto* v_dst = i420.ptr<unsigned char>(height + (height / 4));

        CopyPlane(decoded_buffer->planes[0], static_cast<int>(height),
                  static_cast<int>(width), y_dst);
        CopyPlane(decoded_buffer->planes[1], static_cast<int>(height / 2),
                  static_cast<int>(width / 2), u_dst);
        CopyPlane(decoded_buffer->planes[2], static_cast<int>(height / 2),
                  static_cast<int>(width / 2), v_dst);

        cv::cvtColor(i420, ptr_->frame_buffer.frame, cv::COLOR_YUV2BGR_I420);
        cv::imwrite("bgr.png", ptr_->frame_buffer.frame);
        // std::abort();
        break;
      }
      case UVC_COLOR_FORMAT_YUYV: {
        LOG(FATAL) << "Unimplemented";
        break;
      }
      default:
        LOG(WARNING) << "Unknown frame format";
        break;
    }
    if (ptr_->frame_buffer.frame.empty()) {
      LOG(WARNING) << "Failed to decode frame from camera "
                   << ptr_->camera_constant_.name;
      ptr_->mutex_.unlock();
      return;
    }
    ptr_->frame_buffer.invalid = false;
    ptr_->frame_buffer.timestamp =
        frc::Timer::GetFPGATimestamp()
            .to<double>();  // TODO: Use more accurate timestamp
    ptr_->frame_index_ = frame->sequence;
    ptr_->mutex_.unlock();
  }
}

UVCCamera::UVCCamera(const CameraConstant& camera_constant,
                     absl::Status& status, std::optional<std::string> log_path)
    : camera_constant_(camera_constant),
      log_path_(std::move(log_path)),
      backup_image_(cv::imread("/bos/constants/dont_worry_about_it.jpg")) {

  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");

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
  res = uvc_start_streaming(device_handle_, &ctrl_, callback, this, 0);
  if (res != 0) {
    status = absl::AbortedError("Unable to start streaming for camera: " +
                                camera_constant.name);
    return;
  }
}

auto UVCCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t copied_timestamped_frame;
  while (frame_index_ == previous_frame_index_) {
    std::this_thread::yield();
  }
  mutex_.lock();
  if (frame_buffer.frame.empty()) {
    backup_image_.copyTo(copied_timestamped_frame.frame);
    copied_timestamped_frame.invalid = true;
    copied_timestamped_frame.timestamp =
        frc::Timer::GetFPGATimestamp().to<double>();
  } else {
    frame_buffer.frame.copyTo(copied_timestamped_frame.frame);
    copied_timestamped_frame.invalid = frame_buffer.invalid;
    copied_timestamped_frame.timestamp = frame_buffer.timestamp;
  }
  mutex_.unlock();
  previous_frame_index_ = frame_index_;
  return copied_timestamped_frame;
}

auto UVCCamera::Restart() -> void {
  uvc_stop_streaming(device_handle_);
  uvc_close(device_handle_);
  uvc_unref_device(device_);

  const char* serial_id = camera_constant_.serial_id.has_value()
                              ? camera_constant_.serial_id.value().c_str()
                              : nullptr;
  uvc_find_device(context_, &device_, 0, 0, serial_id);
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
  LOG(INFO) << camera_constant_.name << " has been destructed";
}

auto UVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

}  // namespace camera
