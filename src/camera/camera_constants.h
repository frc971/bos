#pragma once
#include <string>

namespace camera {

typedef struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
} camera_constant_t;

enum CamConstant {
  USB1 = 0,
  USB2 = 1,
};

inline const camera_constant_t a[2] = {
    [CamConstant::USB1] = camera_constant_t{.pipeline = "1",
                                            .intrinsics_path = "",
                                            .extrinsics_path = ""},
    [CamConstant::USB2] = camera_constant_t{.pipeline = "1", .intrinsics_path = "", .extrinsics_path = ""}};

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
