#include "src/camera/uvc_frame_callback.h"

#include <exception>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>

#include "src/utils/pch.h"

namespace camera {
namespace {

constexpr cv::ImreadModes kReadType = cv::IMREAD_GRAYSCALE;

auto DecodeFrame(uvc_frame_t* frame) -> cv::Mat {
  switch (frame->frame_format) {
    case UVC_FRAME_FORMAT_MJPEG: {
      if (frame->data == nullptr || frame->data_bytes == 0U) {
        return {};
      }
      const auto* begin = static_cast<const uchar*>(frame->data);
      const std::vector<uchar> buffer(begin, begin + frame->data_bytes);
      return cv::imdecode(buffer, kReadType);
    }
    case UVC_FRAME_FORMAT_YUYV: {
      if (frame->data == nullptr || frame->width == 0U || frame->height == 0U ||
          (frame->width & 1U) != 0U ||
          frame->width >
              static_cast<std::uint32_t>(std::numeric_limits<int>::max() / 3) ||
          frame->height >
              static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
          frame->width >
              std::numeric_limits<std::size_t>::max() / frame->height / 3U ||
          frame->width >
              std::numeric_limits<std::size_t>::max() / frame->height / 2U ||
          frame->data_bytes <
              static_cast<std::size_t>(frame->width) * frame->height * 2U) {
        return {};
      }
      uvc_frame_t* bgr = uvc_allocate_frame(
          static_cast<std::size_t>(frame->width) * frame->height * 3U);
      if (bgr == nullptr) {
        return {};
      }
      if (uvc_yuyv2bgr(frame, bgr) != UVC_SUCCESS) {
        uvc_free_frame(bgr);
        return {};
      }
      IplImage* image = cvCreateImageHeader(
          cvSize(static_cast<int>(bgr->width), static_cast<int>(bgr->height)),
          IPL_DEPTH_8U, 3);
      if (image == nullptr) {
        uvc_free_frame(bgr);
        return {};
      }
      cvSetData(image, bgr->data, static_cast<int>(bgr->width * 3U));
      cv::Mat decoded = cv::cvarrToMat(image, true);
      cvReleaseImageHeader(&image);
      uvc_free_frame(bgr);
      return decoded;
    }
    default:
      return {};
  }
}

}  // namespace

void UVCFrameCallback(uvc_frame_t* frame, void* receiver) {
  if (frame != nullptr && receiver != nullptr) {
    static_cast<UVCFrameCallbackReceiver*>(receiver)->HandleFrame(frame);
  }
}

void UVCFrameCallbackReceiver::LogCompressedFrame(uvc_frame_t* frame) const {
  if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG ||
      !log_path_.has_value() || log_frequency_ <= 0 || frame->data == nullptr ||
      frame->data_bytes == 0U ||
      frame->sequence % static_cast<std::uint32_t>(log_frequency_) != 0U) {
    return;
  }

  const std::filesystem::path file_path =
      std::filesystem::path(*log_path_) /
      (camera_constant_.name + "_frame_" + std::to_string(frame->sequence) +
       ".jpg");
  std::ofstream file(file_path, std::ios::binary);
  if (!file) {
    LOG(WARNING) << "Failed to open camera frame log " << file_path;
    return;
  }
  file.write(static_cast<const char*>(frame->data),
             static_cast<std::streamsize>(frame->data_bytes));
}

void UVCFrameCallbackReceiver::HandleFrame(uvc_frame_t* frame) {
  std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    ++drops_;
    return;
  }
  ++attempts_;
  LogCompressedFrame(frame);
  cv::Mat decoded;
  try {
    decoded = DecodeFrame(frame);
  } catch (const std::exception& error) {
    ++decode_failures_;
    LOG(WARNING) << "Exception decoding frame from camera "
                 << camera_constant_.name << ": " << error.what();
    return;
  } catch (...) {
    ++decode_failures_;
    LOG(WARNING) << "Unknown exception decoding frame from camera "
                 << camera_constant_.name;
    return;
  }
  if (decoded.empty()) {
    ++decode_failures_;
    LOG(WARNING) << "Failed to decode frame from camera "
                 << camera_constant_.name;
    return;
  }

  frame_.frame = std::move(decoded);
  frame_.invalid = false;
  frame_.timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  ++successes_;
  lock.unlock();
  frame_available_.notify_all();
}

void UVCFrameCallbackReceiver::WaitForFrameAfter(std::uint64_t publication) {
  std::unique_lock lock(mutex_);
  frame_available_.wait(
      lock, [this, publication] { return successes_.load() != publication; });
}

auto UVCFrameCallbackReceiver::CopyLatest(timestamped_frame_t* frame,
                                          std::uint64_t* publication) -> bool {
  std::lock_guard lock(mutex_);
  if (frame_.frame.empty()) {
    return false;
  }
  frame_.frame.copyTo(frame->frame);
  frame->timestamp = frame_.timestamp;
  frame->invalid = frame_.invalid;
  if (publication != nullptr) {
    *publication = successes_.load();
  }
  return true;
}

auto UVCFrameCallbackReceiver::Publication() const -> std::uint64_t {
  return successes_.load();
}

auto UVCFrameCallbackReceiver::Statistics() const
    -> UVCFrameCallbackStatistics {
  return {.attempts = attempts_.load(),
          .successes = successes_.load(),
          .decode_failures = decode_failures_.load(),
          .drops = drops_.load()};
}

}  // namespace camera
