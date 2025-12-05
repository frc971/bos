#pragma once
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>
#include "src/camera/camera.h"
#include "src/camera/camera_source.h"

namespace camera {

bool WriteFrame(std::string folder, timestamped_frame_t& timestamped_frame);

}  // namespace camera
