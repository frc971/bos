#pragma once
#include <frc/geometry/Transform3d.h>
#include "src/utils/pch.h"

namespace utils {

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;
}
