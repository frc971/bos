#pragma once
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>
#include "src/camera/camera.h"
#include "src/camera/camera_source.h"

namespace camera {

bool WriteFrame(timestamped_frame_t& frame);

}  // namespace camera
