#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <vector>
#include "nlohmann/json_fwd.hpp"
#include "src/localization/position.h"
namespace localization {

static frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2025ReefscapeAndyMark);

auto ToFeildRelitivePosition(
    std::vector<position_estimate_t> detections,
    frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> std::vector<position_estimate_t>;

auto ToFeildRelitivePosition(
    position_estimate_t tag_relative_position, frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> position_estimate_t;

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;

}  // namespace localization
