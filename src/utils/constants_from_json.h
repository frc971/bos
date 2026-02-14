#pragma once

#include <frc/geometry/Transform3d.h>
#include "src/utils/pch.h"

namespace utils {

template <typename T>
auto CameraMatrixFromJson(nlohmann::json intrinsics) -> T;

template <typename T>
auto DistortionCoefficientsFromJson(nlohmann::json intrinsics) -> T;

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;

}  // namespace utils
