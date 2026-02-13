#pragma once

#include <frc/geometry/Transform3d.h>
#include "src/utils/pch.h"

namespace utils {

template <typename T>
auto camera_matrix_from_json(nlohmann::json intrinsics) -> T;

template <typename T>
auto distortion_coefficients_from_json(nlohmann::json intrinsics) -> T;

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;

}  // namespace utils
