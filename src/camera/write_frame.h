#pragma once
#include "src/camera/camera.h"
#include "src/camera/camera_source.h"
#include "src/utils/pch.h"

namespace camera {

auto WriteFrame(const std::string& folder,
                timestamped_frame_t& timestamped_frame) -> bool;

}  // namespace camera
