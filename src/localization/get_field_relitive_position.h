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
    std::vector<tag_detection_t> detections, frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> std::vector<tag_detection_t>;

auto ToFeildRelitivePosition(
    tag_detection_t tag_relative_position, frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> tag_detection_t;

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;

}  // namespace localization
