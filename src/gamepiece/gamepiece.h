#pragma once
#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include <nlohmann/json.hpp>
#include "src/camera/camera_source.h"
#include "src/yolo/yolo.h"

namespace gamepiece {
void run_gamepiece_detect(yolo::Yolo& model,
                          const std::vector<std::string>& class_names,
                          camera::CameraSource& source,
                          nt::StructTopic<frc::Pose2d>& coral_topic,
                          nt::StructTopic<frc::Pose2d>& algae_topic,
                          nlohmann::json intrinsics, nlohmann::json extrinsics,
                          bool debug);
}  // namespace gamepiece
