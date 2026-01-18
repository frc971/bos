#pragma once

#include <string>
namespace camera {

using camera_constant_t = struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
  std::string name;
};

enum Camera {
  IMX296_0,
  IMX296_1,

  FIDDLER_USB0,
  FIDDLER_USB1,

  TURRET_BOT_FRONT_RIGHT,
  TURRET_BOT_FRONT_LEFT,
  TURRET_BOT_BACK_RIGHT,
  TURRET_BOT_BACK_LEFT,

  VISION_BOT_FRONT_RIGHT,
  VISION_BOT_FRONT_LEFT,
  VISION_BOT_BACK_RIGHT,
  VISION_BOT_BACK_LEFT,

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
            .extrinsics_path = "/bos/constants/imx296_camera0_extrinsics.json",
            .name = "mipi0"},
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
            .extrinsics_path = "/bos/constants/imx296_camera1_extrinsics.json", 
            .name = "mipi1"},
    [Camera::FIDDLER_USB0] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.1:1.0-video-index0",
            .intrinsics_path = "/bos/constants/fiddler_usb_camera0_intrinsics.json",
            .extrinsics_path = "/bos/constants/fiddler_usb_camera0_extrinsics.json", 
            .name = "fiddler_usb0"},
    [Camera::FIDDLER_USB1] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/fiddler_usb_camera1_intrinsics.json",
            .extrinsics_path = "/bos/constants/fiddler_usb_camera1_extrinsics.json", 
            .name = "fiddler_usb1"},
    [Camera::TURRET_BOT_FRONT_RIGHT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.2:1.0-video-index0",
            .intrinsics_path = "/bos/constants/turret_bot/front_right_intrinsics.json",
            .extrinsics_path = "/bos/constants/turret_bot/front_right_extrinsics.json", 
            .name = "turret_bot_front_right"},
    [Camera::TURRET_BOT_FRONT_LEFT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/turret_bot/front_left_intrinsics.json",
            .extrinsics_path = "/bos/constants/turret_bot/front_left_extrinsics.json", 
            .name = "turret_bot_front_left"},
    [Camera::TURRET_BOT_BACK_RIGHT] =
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
            .intrinsics_path = "/bos/constants/turret_bot/back_right_intrinsics.json",
            .extrinsics_path = "/bos/constants/turret_bot/back_right_extrinsics.json", 
            .name = "turret_bot_back_right"},
  [Camera::TURRET_BOT_BACK_LEFT] =
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
            .intrinsics_path = "/bos/constants/turret_bot/back_left_intrinsics.json",
            .extrinsics_path = "/bos/constants/turret_bot/back_left_extrinsics.json", 
          .name = "turret_bot_back_left"},
    [Camera::VISION_BOT_FRONT_RIGHT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/vision_bot_front_right_intrinsics.json",
            .extrinsics_path = "/bos/constants/vision_bot_front_right_extrinsics.json", 
            .name = "vision_bot_front_right"},
    [Camera::VISION_BOT_FRONT_LEFT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/vision_bot_front_left_intrinsics.json",
            .extrinsics_path = "/bos/constants/vision_bot_front_left_extrinsics.json", 
            .name = "vision_bot_front_leftb"},
    [Camera::VISION_BOT_BACK_RIGHT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/vision_bot_back_right_intrinsics.json",
            .extrinsics_path = "/bos/constants/vision_bot_back_right_extrinsics.json",
            .name = "turret_bot_back_right"},
    [Camera::VISION_BOT_BACK_LEFT] =
        camera_constant_t{
            .pipeline = "/dev/v4l/by-path/"
                        "platform-3610000.usb-usb-0:2.4:1.0-video-index0",
            .intrinsics_path = "/bos/constants/vision_bot_back_left_intrinsics.json",
            .extrinsics_path = "/bos/constants/vision_bot_back_left_extrinsics.json", 
            .name = "vision_bot_back_left"},
    [Camera::DEFAULT_USB0] =
        camera_constant_t{
            .pipeline = "platform-3610000.usb-usb-0:2.2:1.0-video-index0",
            .intrinsics_path =
                "/bos/constants/default_usb_camera0_intrinsics.json",
            .extrinsics_path =
                "/bos/constants/default_usb_camera0_extrinsics.json",
            .name = "default_usb0"},
    [Camera::REALSENSE] =
        camera_constant_t{
            .pipeline = "",
            .intrinsics_path = "/bos/constants/realsense_intrinsics.json",
            .extrinsics_path = "/bos/constants/realsense_extrinsics.json",
            .name = "realsense"},
};
}; // namespace camera
