#include "src/camera/uvc_camera.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include "/usr/src/jetson_multimedia_api/include/NvBuffer.h"
#include "/usr/src/jetson_multimedia_api/include/NvUtils.h"
#include "absl/status/status.h"
#include "src/utils/pch.h"

namespace camera {

const cv::Mat UVCCamera::backup_image_ =
    cv::imread("/bos/constants/dont_worry_about_it.jpg");

void FreeNvBuffer(NvBuffer* buff) {
  if (buff != nullptr) {
    buff->unmap();
    delete buff;
  }
}

void callback(uvc_frame_t* frame, void* ptr) {
  LOG(INFO) << "in cb";
  auto ptr_ = static_cast<UVCCamera*>(ptr);
  ptr_->mutex_.lock();
  switch (frame->frame_format) {
    case UVC_COLOR_FORMAT_MJPEG: {
      LOG(INFO) << "1";
      NvBuffer* decoded_frame_buffer = nullptr;
      uint32_t pixfmt, width, height;

      int ret = ptr_->decoder_->decodeToBuffer(
          &decoded_frame_buffer, reinterpret_cast<unsigned char*>(frame->data),
          frame->data_bytes, &pixfmt, &width, &height);
      LOG(INFO) << "2";

      if (ret != 0 || decoded_frame_buffer == nullptr) {
        LOG(WARNING) << "Decode failed for camera "
                     << ptr_->camera_constant_.name << " with code: " << ret;
        ptr_->mutex_.unlock();
        FreeNvBuffer(decoded_frame_buffer);
        return;
      }
      LOG(INFO) << "3";

      if (decoded_frame_buffer->map() != 0) {
        LOG(WARNING) << "Failed to map NvBuffer for camera "
                     << ptr_->camera_constant_.name << " with code: " << ret;
        ptr_->mutex_.unlock();
        FreeNvBuffer(decoded_frame_buffer);
        return;
      }
      LOG(INFO) << "4";

      if (ptr_->read_type == cv::IMREAD_COLOR) {
        cv::Mat yuv_mat(height * 3 / 2, width, CV_8UC1);

        for (uint32_t plane = 0; plane < decoded_frame_buffer->n_planes;
             plane++) {
          NvBuffer::NvBufferPlane& nv_plane =
              decoded_frame_buffer->planes[plane];

          uint8_t* src = nv_plane.data;
          uint8_t* dst = yuv_mat.data + (plane == 0 ? 0 : width * height);

          for (uint32_t row = 0; row < nv_plane.fmt.height; row++) {
            memcpy(dst + row * nv_plane.fmt.width,
                   src + row * nv_plane.fmt.bytesperpixel * nv_plane.fmt.stride,
                   nv_plane.fmt.width * nv_plane.fmt.bytesperpixel);
          }
        }
        ptr_->frame_buffer.frame = std::move(yuv_mat);
      } else {
        LOG(INFO) << "5";
        NvBuffer::NvBufferPlane& luminance = decoded_frame_buffer->planes[0];
        ptr_->frame_buffer.frame =
            cv::Mat(height, width, CV_8UC1, luminance.data,
                    luminance.fmt.stride * luminance.fmt.bytesperpixel)
                .clone();
        LOG(INFO) << "6";
      }
      LOG(INFO) << "7";
      ptr_->mutex_.unlock();
      FreeNvBuffer(decoded_frame_buffer);
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
  LOG(INFO) << "out cb";
}

UVCCamera::UVCCamera(const CameraConstant& camera_constant,
                     absl::Status& status, std::optional<std::string> log_path)
    : camera_constant_(camera_constant),
      log_path_(std::move(log_path)),
      decoder_(
          NvJPEGDecoder::createJPEGDecoder(camera_constant_.name.c_str())) {
  LOG(INFO) << "In constructor";
  if (decoder_ == nullptr) {
    status = absl::InternalError("Failed to create JPEG decoder for " +
                                 camera_constant_.name);
    return;
  }
  LOG(INFO) << "After decode";
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
  delete decoder_;
}

auto UVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

}  // namespace camera
