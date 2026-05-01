#include "src/camera/uvc_camera.h"
#include "/usr/src/jetson_multimedia_api/include/NvBuffer.h"
#include "absl/status/status.h"
#include "src/utils/pch.h"

namespace camera {

namespace {

constexpr unsigned char kJpegStartMarker[2] = {0xFF, 0xD8};
constexpr unsigned char kJpegEndMarker[2] = {0xFF, 0xD9};

auto NormalizeJpegBuffer(const unsigned char* data, size_t size)
    -> std::vector<unsigned char> {
  if (data == nullptr || size < 2) {
    return {};
  }

  const unsigned char* begin = data;
  const unsigned char* end = data + size;

  const unsigned char* soi =
      std::search(begin, end, std::begin(kJpegStartMarker),
                  std::end(kJpegStartMarker));
  if (soi == end) {
    return {};
  }

  const unsigned char* eoi = end;
  for (const unsigned char* current = end - 2; current >= soi; --current) {
    if (current[0] == kJpegEndMarker[0] && current[1] == kJpegEndMarker[1]) {
      eoi = current + 2;
      break;
    }
    if (current == soi) {
      break;
    }
  }

  std::vector<unsigned char> normalized(soi, eoi == end ? end : eoi);
  if (normalized.size() < 2) {
    return {};
  }

  if (eoi == end && (normalized[normalized.size() - 2] != kJpegEndMarker[0] ||
                     normalized[normalized.size() - 1] != kJpegEndMarker[1])) {
    normalized.push_back(kJpegEndMarker[0]);
    normalized.push_back(kJpegEndMarker[1]);
  }

  return normalized;
}

void FreeNvBuffer(NvBuffer* buff) {
  if (buff != nullptr) {
    buff->unmap();
    delete buff;
  }
}

}  // namespace

void callback(uvc_frame_t* frame, void* ptr) {
  auto ptr_ = static_cast<UVCCamera*>(ptr);
  std::unique_lock<std::mutex> lock(ptr_->mutex_);

  switch (frame->frame_format) {
    case UVC_COLOR_FORMAT_MJPEG: {
      NvBuffer* decoded_frame_buffer = nullptr;
      uint32_t pixfmt, width, height;
      std::vector<unsigned char> jpeg =
          NormalizeJpegBuffer(reinterpret_cast<unsigned char*>(frame->data),
                              frame->data_bytes);
      if (jpeg.empty()) {
        LOG(WARNING) << "Failed to normalize MJPEG frame for camera "
                     << ptr_->camera_constant_.name;
        return;
      }

      int ret = ptr_->decoder_->decodeToBuffer(
          &decoded_frame_buffer, jpeg.data(), jpeg.size(), &pixfmt, &width,
          &height);

      if (ret != 0 || decoded_frame_buffer == nullptr) {
        LOG(WARNING) << "Decode failed for camera "
                     << ptr_->camera_constant_.name << " with code: " << ret;
        FreeNvBuffer(decoded_frame_buffer);
        return;
      }

      ret = decoded_frame_buffer->map();
      if (ret != 0) {
        LOG(WARNING) << "Failed to map NvBuffer for camera "
                     << ptr_->camera_constant_.name << " with code: " << ret;
        FreeNvBuffer(decoded_frame_buffer);
        return;
      }

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
        ptr_->frame_buffer.frame = yuv_mat.clone();
      } else {
        NvBuffer::NvBufferPlane& luminance = decoded_frame_buffer->planes[0];
        ptr_->frame_buffer.frame =
            cv::Mat(height, width, CV_8UC1, luminance.data,
                    luminance.fmt.stride * luminance.fmt.bytesperpixel)
                .clone();
      }
      FreeNvBuffer(decoded_frame_buffer);
      break;
    }
    case UVC_COLOR_FORMAT_YUYV: {
      uvc_frame_t* bgr = uvc_allocate_frame(frame->width * frame->height * 3);
      if (!bgr) {
        LOG(WARNING) << "Camera " << ptr_->camera_constant_.name
                     << " failed to allocate ";
        return;
      }
      uvc_error_t ret = uvc_yuyv2bgr(frame, bgr);
      if (ret != 0) {
        LOG(WARNING) << "YUYV failed to convert to BGR";
        uvc_free_frame(bgr);
        return;
      }
      ptr_->frame_buffer.frame =
          cv::Mat(bgr->height, bgr->width, CV_8UC3, bgr->data, bgr->width * 3)
              .clone();
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
}

UVCCamera::UVCCamera(const CameraConstant& camera_constant,
                     absl::Status& status, std::optional<std::string> log_path)
    : camera_constant_(camera_constant),
      log_path_(std::move(log_path)),
      decoder_(
          NvJPEGDecoder::createJPEGDecoder(camera_constant_.name.c_str())) {
  if (decoder_ == nullptr) {
    status = absl::InternalError("Failed to create JPEG decoder for " +
                                 camera_constant_.name);
    return;
  }
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_buffer.frame.empty()) {
    copied_timestamped_frame.invalid = true;
    copied_timestamped_frame.timestamp =
        frc::Timer::GetFPGATimestamp().to<double>();
  } else {
    frame_buffer.frame.copyTo(copied_timestamped_frame.frame);
    copied_timestamped_frame.invalid = frame_buffer.invalid;
    copied_timestamped_frame.timestamp = frame_buffer.timestamp;
  }
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
  if (device_handle_ != nullptr) {
    uvc_stop_streaming(device_handle_);
    uvc_close(device_handle_);
  }
  if (device_ != nullptr) {
    uvc_unref_device(device_);
  }
  if (context_ != nullptr) {
    uvc_exit(context_);
  }
  delete decoder_;
}

auto UVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

}  // namespace camera
