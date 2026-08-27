#include "src/camera/simulated_uvc_camera.h"

struct uvc_context {
  camera::test::MockUvcApi* mock;
};
struct uvc_device {
  camera::test::MockUvcApi* mock;
};
struct uvc_device_handle {
  camera::test::MockUvcApi* mock;
};

extern "C" {

uvc_error_t uvc_init(uvc_context_t** context, libusb_context*) {
  auto* mock = camera::test::internal::constructing_mock;
  if (mock == nullptr)
    return UVC_ERROR_INVALID_PARAM;
  const auto result = mock->Init();
  if (result != UVC_SUCCESS)
    return result;
  *context = new uvc_context_t{mock};
  return UVC_SUCCESS;
}

void uvc_exit(uvc_context_t* context) {
  context->mock->Exit(context);
  delete context;
}

uvc_error_t uvc_find_device(uvc_context_t* context, uvc_device_t** device, int,
                            int, const char*) {
  const auto result = context->mock->FindDevice();
  if (result == UVC_SUCCESS)
    *device = new uvc_device_t{context->mock};
  return result;
}

uvc_error_t uvc_open(uvc_device_t* device, uvc_device_handle_t** handle) {
  const auto result = device->mock->Open();
  if (result == UVC_SUCCESS)
    *handle = new uvc_device_handle_t{device->mock};
  return result;
}

void uvc_close(uvc_device_handle_t* handle) {
  handle->mock->Close(handle);
  delete handle;
}

void uvc_unref_device(uvc_device_t* device) {
  device->mock->UnrefDevice(device);
  delete device;
}

uvc_error_t uvc_get_stream_ctrl_format_size(uvc_device_handle_t* handle,
                                            uvc_stream_ctrl_t* ctrl,
                                            uvc_frame_format, int width,
                                            int height, int fps) {
  return handle->mock->GetStreamControl(ctrl, width, height, fps);
}

void uvc_print_stream_ctrl(uvc_stream_ctrl_t*, FILE*) {}

uvc_error_t uvc_start_streaming(uvc_device_handle_t* handle, uvc_stream_ctrl_t*,
                                uvc_frame_callback_t* callback, void* user,
                                uint8_t) {
  return handle->mock->StartStreaming(callback, user);
}

void uvc_stop_streaming(uvc_device_handle_t* handle) {
  handle->mock->StopStreaming(handle);
}

uvc_frame_t* uvc_allocate_frame(size_t bytes) {
  auto* frame = new uvc_frame_t{};
  frame->data = new unsigned char[bytes];
  frame->data_bytes = bytes;
  return frame;
}

void uvc_free_frame(uvc_frame_t* frame) {
  if (frame == nullptr)
    return;
  delete[] static_cast<unsigned char*>(frame->data);
  delete frame;
}

uvc_error_t uvc_yuyv2bgr(uvc_frame_t*, uvc_frame_t*) {
  return UVC_ERROR_NOT_SUPPORTED;
}

}  // extern "C"
