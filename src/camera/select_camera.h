#pragma once
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cv_camera.h"

namespace camera {
auto SelectCameraConfig() -> Camera;
auto SelectCameraConfig(const std::string& choice) -> Camera;
auto GetCameraStream(Camera camera) -> std::unique_ptr<ICamera>;
}  // namespace camera
