#pragma once
#include <string>

namespace camera {

using camera_constant_t = struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
};

enum Camera {
  IMX296_0,
  IMX296_1,
  FIDDLER_USB0,
  FIDDLER_USB1,
  SOFTWARE_BOT_USB0,
  SOFTWARE_BOT_USB1,
  DEFAULT_USB0,
  REALSENSE,
  CAMERA_LENGTH,
};

inline const camera_constant_t camera_constants[CAMERA_LENGTH] = {
    [Camera::IMX296_0] =
        camera_constant_t{
            .pipeline =
                "nvarguscamerasrc sensor-id=0 "
                "aelock=true exposuretimerange=\"100000 "
                "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                "video/x-raw(memory:NVMM), width=1456, height=1088, "
                "framerate=30/1, "
                "format=NV12 ! "
                "nvvidconv ! "
                "video/x-raw, format=BGRx ! "
                "queue ! "
                "appsink",
            .intrinsics_path = "/bos/constants/imx296_camera0_intrinsics.json",
            .extrinsics_path = "/bos/constants/imx296_camera0_extrinsics.json"},
    [Camera::IMX296_1] =
        camera_constant_t{
            .pipeline =
                "nvarguscamerasrc sensor-id=1 "
                "aelock=true exposuretimerange=\"100000 "
                "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                "video/x-raw(memory:NVMM), width=1456, height=1088, "
                "framerate=30/1, "
                "format=NV12 ! "
                "nvvidconv ! "
                "video/x-raw, format=BGRx ! "
                "queue ! "
                "appsink",
            .intrinsics_path = "/bos/constants/imx296_camera1_intrinsics.json",
            .extrinsics_path = "/bos/constants/imx296_camera1_extrinsics.json"},
    [Camera::FIDDLER_USB0] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.1:1.0-video-index0",
            .intrinsics_path = "/bos/constants/fiddler_usb_camera0_intrinsics.json",
            .extrinsics_path = "/bos/constants/fiddler_usb_camera0_extrinsics.json"},
    [Camera::FIDDLER_USB1] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/fiddler_usb_camera1_intrinsics.json",
            .extrinsics_path = "/bos/constants/fiddler_usb_camera1_extrinsics.json"},
    [Camera::SOFTWARE_BOT_USB0] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.1:1.0-video-index0",
            .intrinsics_path = "/bos/constants/software_bot_usb_camera0_intrinsics.json",
            .extrinsics_path = "/bos/constants/software_bot_usb_camera0_extrinsics.json"},
    [Camera::SOFTWARE_BOT_USB1] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/software_bot_usb_camera1_intrinsics.json",
            .extrinsics_path = "/bos/constants/software_bot_usb_camera1_extrinsics.json"},
    [Camera::DEFAULT_USB0] =
        camera_constant_t{
            .pipeline = "/dev/video0",
            .intrinsics_path =
                "/bos/constants/default_usb_camera0_intrinsics.json",
            .extrinsics_path =
                "/bos/constants/default_usb_camera0_extrinsics.json"},
    [Camera::REALSENSE] =
        camera_constant_t{
            .pipeline = "",
            .intrinsics_path = "/bos/constants/realsense_intrinsics.json",
            .extrinsics_path = "/bos/constants/realsense_extrinsics.json"},
};
}; // namespace camera
