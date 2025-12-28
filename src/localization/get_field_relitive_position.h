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

std::vector<tag_detection_t> GetFeildRelitivePosition(
    std::vector<tag_detection_t> detections, nlohmann::json extrinsics,
    frc::AprilTagFieldLayout apriltag_layout = kapriltag_layout,
    bool verbose = false);

tag_detection_t GetFeildRelitivePosition(
    tag_detection_t tag_relative_position, nlohmann::json extrinsics,
    frc::AprilTagFieldLayout apriltag_layout = kapriltag_layout,
    bool verbose = false);
}  // namespace localization
