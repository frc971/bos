#pragma once
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cv_camera.h"
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
auto SelectCameraConfig() -> Camera;
auto SelectCameraConfig(const std::string& choice) -> Camera;
auto SelectCameraConfig(std::optional<std::string> choice) -> Camera;
auto GetCameraStream(Camera camera) -> std::unique_ptr<ICamera>;
}  // namespace camera
