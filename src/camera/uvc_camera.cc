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
  ptr_->mutex_.lock();
  switch (frame->frame_format) {
    case UVC_COLOR_FORMAT_MJPEG: {
      char* data = static_cast<char*>(frame->data);
      std::vector<uchar> buffer(data, data + frame->data_bytes);
      ptr_->frame_buffer.frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
      break;
    }
    case UVC_COLOR_FORMAT_YUYV: {
      uvc_frame_t* bgr = uvc_allocate_frame(frame->width * frame->height * 3);
      if (!bgr) {
        LOG(WARNING) << "Camera " << ptr_->camera_constant_.name
                     << " failed to allocate ";
      }
      uvc_error_t ret = uvc_yuyv2bgr(frame, bgr);
      if (ret != 0) {
        LOG(WARNING) << "YUYV failed to convert to BGR";
      }
      IplImage* ipl_image;
      ipl_image =
          cvCreateImageHeader(cvSize(bgr->width, bgr->height), IPL_DEPTH_8U, 3);
      cvSetData(ipl_image, bgr->data, bgr->width * 3);
      ptr_->frame_buffer.frame = cv::cvarrToMat(ipl_image, true);
      uvc_free_frame(bgr);
      break;
    }
    default:
      LOG(WARNING) << "Unknown frame format";
      break;
  }
  if (ptr_->frame_buffer.frame.empty()) {
    LOG(WARNING) << "Failed to decode frame from camera "
                 << ptr_->camera_constant_.name;
    return;
  }
  ptr_->frame_buffer.invalid = false;
  ptr_->frame_buffer.timestamp =
      frc::Timer::GetFPGATimestamp()
          .to<double>();  // TODO: Use more accurate timestamp
  ptr_->frame_index_ = frame->sequence;
  ptr_->mutex_.unlock();
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
    status = absl::AbortedError("Unable to get stream format for camera: " +
                                camera_constant.name);
    return;
  }
  uvc_print_stream_ctrl(&ctrl_, stderr);
  ctrl_.dwMaxPayloadTransferSize = 1024;
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
