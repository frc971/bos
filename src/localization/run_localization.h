#pragma once
#include <string>
#include "src/camera/camera_source.h"
namespace localization {
void run_localization(const int frame_width, const int frame_height,
                      camera::CameraSource& source, std::string intrinsics,
                      std::string extrinsics, uint port, bool verbose = false);
}
