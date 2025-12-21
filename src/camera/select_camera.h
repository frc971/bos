#pragma once
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cv_camera.h"

namespace camera {
Camera SelectCameraConfig();
std::unique_ptr<ICamera> GetCameraStream(Camera camera);
}  // namespace camera
