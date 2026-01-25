#pragma once
#include "src/camera/camera_source.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position_solver.h"
#include "src/utils/pch.h"

namespace localization {
void run_localization(camera::CameraSource& source,
                      std::unique_ptr<localization::IAprilTagDetector> detector,
                      std::unique_ptr<localization::IPositionSolver> solver,
                      const std::string& extrinsics, uint port, bool verbose);
}
