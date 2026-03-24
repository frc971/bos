#pragma once
#include <optional>
#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cv_camera.h"
#include "src/utils/pch.h"

ABSL_DECLARE_FLAG(std::optional<std::string>, folder_name);  //NOLINT

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

auto GetCameraStream(camera_constant_t camera) -> std::unique_ptr<ICamera>;
}  // namespace camera
