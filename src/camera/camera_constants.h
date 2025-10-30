#pragma once
#include <string>

namespace camera {

typedef struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
} camera_constant_t;

enum Camera {
  IMX296_0 = 0,
  IMX296_1 = 1,
  USB0 = 2,
  USB1 = 3,
};

inline const camera_constant_t camera_constants[4] = {
    [Camera::IMX296_0] = camera_constant_t{.pipeline = "nvarguscamerasrc sensor-id=0 " 
                                                        "aelock=true exposuretimerange=\"100000 "
                                                        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                                                        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
                                                        "format=NV12 ! "
                                                        "nvvidconv ! "
                                                        "video/x-raw, format=BGRx ! "
                                                        "queue ! "
                                                        "appsink",                        
                                                .intrinsics_path = "/bos/constants/imx296_camera0_intrinsics.json",
                                                .extrinsics_path = "/bos/constants/imx296_camera0_extrinsics.json"},
    [Camera::IMX296_1] = camera_constant_t{.pipeline = "nvarguscamerasrc sensor-id=1 " 
                                                        "aelock=true exposuretimerange=\"100000 "
                                                        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                                                        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
                                                        "format=NV12 ! "
                                                        "nvvidconv ! "
                                                        "video/x-raw, format=BGRx ! "
                                                        "queue ! "
                                                        "appsink",
                                                .intrinsics_path = "/bos/constants/imx296_camera1_intrinsics.json", 
                                                .extrinsics_path = "/bos/constants/imx296_camera1_extrinsics.json"},
  [Camera::USB0] = camera_constant_t{.pipeline = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
    .intrinsics_path = "/bos/constants/usb_camera0_intrinsics.json",
    .extrinsics_path = "/bos/constants/usb_camera0_extrinsics.json"},
  [Camera::USB1] = camera_constant_t{.pipeline = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0", 
    .intrinsics_path = "/bos/constants/usb_camera1_intrinsics.json", 
    .extrinsics_path = "/bos/constants/usb_camera1_extrinsics.json"},
};
};  // namespace camera
