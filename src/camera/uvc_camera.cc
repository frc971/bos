#include "src/camera/uvc_camera.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include "absl/status/status.h"
#include "src/utils/pch.h"

namespace camera {
// void frame_callback(uvc_frame_t* frame, void* ptr, const camera_constant_t& ) {
//   uvc_frame_t* bgr;
//   uvc_error_t ret;
//   IplImage* ipl_image;
//
//   bgr = uvc_allocate_frame(frame->width * frame->height * 3);
//   if (!bgr) {
//     LOG(WARNING) << "Camera "
//   }
// }

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

  uvc_stream_ctrl_t ctrl;
  res = uvc_get_stream_ctrl_format_size(
      device_handle_, &ctrl, UVC_FRAME_FORMAT_MJPEG,
      camera_constant_.frame_width.value(),
      camera_constant_.frame_height.value(), camera_constant_.fps.value());
  if (res != 0) {
    status = absl::AbortedError("Unable to create context for camera: " +
                                camera_constant.name);
    return;
  }
  uvc_print_stream_ctrl(&ctrl, stderr);
  res = uvc_start_streaming(
      device_handle_, &ctrl,
      [](uvc_frame_t* frame, void* ptr) {
        auto ptr_ = static_cast<UVCCamera*>(ptr);
        cv::Mat img;
        switch (frame->frame_format) {
          case UVC_COLOR_FORMAT_MJPEG: {
            char* data = static_cast<char*>(frame->data);
            std::vector<uchar> buffer(data, data + frame->data_bytes);
            img = cv::imdecode(buffer, cv::IMREAD_GRAYSCALE);
            break;
          }
          case UVC_COLOR_FORMAT_YUYV: {
            uvc_frame_t* bgr =
                uvc_allocate_frame(frame->width * frame->height * 3);
            if (!bgr) {
              LOG(WARNING) << "Camera " << ptr_->camera_constant_.name
                           << " failed to allocate ";
            }
            uvc_error_t ret = uvc_yuyv2bgr(frame, bgr);
            if (ret != 0) {
              LOG(WARNING) << "YUYV failed to convert to BGR";
            }
            IplImage* ipl_image;
            ipl_image = cvCreateImageHeader(cvSize(bgr->width, bgr->height),
                                            IPL_DEPTH_8U, 3);
            cvSetData(ipl_image, bgr->data, bgr->width * 3);
            img = cv::cvarrToMat(ipl_image, true);
            break;
          }
          default:
            break;
        }
        if (img.empty()) {
          LOG(WARNING) << "Failed to decode frame from camera "
                       << ptr_->camera_constant_.name;
        }
      },
      this, 0);

  // res = uvc_start_streaming(
  //     device_handle_, &ctrl, [](uvc_frame_t* frame, void* ptr) {}, nullptr,
  //     this, 0);  // this number very important, don't change
  if (res != 0) {
    status = absl::AbortedError("Unable to create context for camera: " +
                                camera_constant.name);
    return;
  }
}
}  // namespace camera
