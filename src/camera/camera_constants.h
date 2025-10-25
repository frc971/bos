#pragma once
#include <string>

namespace camera {

typedef struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
} camera_constant_t;

enum Cameras {
  USB0 = 0,
  USB1 = 1,
  IMX296_0 = 2,
  IMX296_1 = 3
};

inline const camera_constant_t camera_constants[4] = {
    [Cameras::USB0] = camera_constant_t{.pipeline = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
                                            .intrinsics_path = "constants/usb_camera0_intrinsics.json",
                                            .extrinsics_path = "constants/usb_camera0_extrinsics.json"},
    [Cameras::USB1] = camera_constant_t{.pipeline = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0", 
                                            .intrinsics_path = "constants/usb_camera1_intrinsics.json", 
                                            .extrinsics_path = "constants/usb_camera1_extrinsics.json"},
    [Cameras::IMX296_0] = camera_constant_t{.pipeline = " nvarguscamerasrc sensor-id=0" 
                                                        " aelock=true exposuretimerange=\"100000 "
                                                        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                                                        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
                                                        "format=NV12 ! "
                                                        "nvvidconv ! "
                                                        "video/x-raw, format=BGRx ! "
                                                        "queue ! "
                                                        "appsink",                        
                                                .intrinsics_path = "constants/imx296_camera1_intrinsics.json",
                                                .extrinsics_path = "constants/imx296_camera1_extrinsics.json"},
    [Cameras::IMX296_1] = camera_constant_t{.pipeline = " nvarguscamerasrc sensor-id=1" 
                                                        " aelock=true exposuretimerange=\"100000 "
                                                        "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
                                                        "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
                                                        "format=NV12 ! "
                                                        "nvvidconv ! "
                                                        "video/x-raw, format=BGRx ! "
                                                        "queue ! "
                                                        "appsink",
                                                .intrinsics_path = "constants/imx296_camera2_intrinsics.json", 
                                                .extrinsics_path = "constants/imx296_camera2_extrinsics.json"}
};

const std::string usb_camera0_intrinsics =
    "constants/usb_camera0_intrinsics.json";
const std::string usb_camera0_extrinsics =
    "constants/usb_camera0_extrinsics.json";

const std::string imx296_camera1_intrinsics =
    "constants/imx296_camera1_intrinsics.json";
const std::string imx296_camera1_extrinsics =
    "constants/imx296_camera1_extrinsics.json";

const std::string imx296_camera2_intrinsics =
    "constants/imx296_camera2_intrinsics.json";
const std::string imx296_camera2_extrinsics =
    "constants/imx296_camera2_extrinsics.json";

const std::string usb_camera1_intrinsics =
    "constants/usb_camera1_intrinsics.json";
const std::string usb_camera1_extrinsics =
    "constants/usb_camera1_extrinsics.json";

const std::string usb_camera2_intrinsics =
    "constants/usb_camera2_intrinsics.json";
const std::string usb_camera2_extrinsics =
    "constants/usb_camera2_extrinsics.json";

const std::string usb_camera0 =
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0";

const std::string usb_camera1 =
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0";

};  // namespace camera
