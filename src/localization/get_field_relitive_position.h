#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include "nlohmann/json_fwd.hpp"
#include "src/localization/position.h"
#include "src/utils/pch.h"
namespace localization {

static frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout("/bos/constants/2026-rebuilt-welded.json");

// returns camera to robot
auto ToFieldRelitivePosition(
    std::vector<tag_detection_t> detections, frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> std::vector<tag_detection_t>;

// returns camera to robot
auto ToFieldRelitivePosition(
    tag_detection_t tag_relative_position, frc::Transform3d camera_to_robot,
    const frc::AprilTagFieldLayout& apriltag_layout = kapriltag_layout,
    bool verbose = false) -> tag_detection_t;

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d;

}  // namespace localization
