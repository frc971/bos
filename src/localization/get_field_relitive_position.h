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

std::vector<tag_detection_t> ToFeildRelitivePosition(
    std::vector<tag_detection_t> detections, frc::Transform3d camera_to_robot,
    frc::AprilTagFieldLayout apriltag_layout = kapriltag_layout,
    bool verbose = false);

tag_detection_t ToFeildRelitivePosition(
    tag_detection_t tag_relative_position, frc::Transform3d camera_to_robot,
    frc::AprilTagFieldLayout apriltag_layout = kapriltag_layout,
    bool verbose = false);

frc::Transform3d ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json);

}  // namespace localization
