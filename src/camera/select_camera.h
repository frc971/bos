#pragma once
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/utils/pch.h"

namespace camera {
// Utility functions to let the user select what camera they want to use with testing.
// If any input is invalid, we ask the user manually what camera they want with std::cin.
// Also supports ABSL flags:
// camera::Camera config =
//     camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));
// camera::CameraSource source("stress_test_camera",
//                             camera::GetCameraStream(config));
//
auto SelectCameraConfig(const camera::camera_constants_t& camera_constants)
    -> camera_constant_t;
auto SelectCameraConfig(const std::string& choice,
                        const camera::camera_constants_t& camera_constants)
    -> camera_constant_t;
auto SelectCameraConfig(std::optional<std::string> choice,
                        const camera::camera_constants_t& camera_constants)
    -> camera_constant_t;

auto SelectCamera(const std::string& name, std::optional<std::string> choice,
                  const camera_constants_t& camera_constants) -> CameraSource;
auto SelectCamera(const std::string& name, std::optional<std::string> choice,
                  const camera_constants_t& camera_constants, double disk_speed)
    -> CameraSource;

// auto GetCameraStream(Camera camera) -> std::unique_ptr<ICamera>;
}  // namespace camera
