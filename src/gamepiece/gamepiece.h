#pragma once
#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include "src/camera/camera_source.h"
#include "src/yolo/yolo.h"
#include <nlohmann/json.hpp>

namespace gamepiece {
void run_gamepiece_detect(yolo::Yolo& model,
                          const std::vector<std::string>& class_names,
                          std::shared_ptr<camera::CameraSource> camera,
                          nt::StructTopic<frc::Pose2d>& coral_topic,
                          nt::StructTopic<frc::Pose2d>& algae_topic,
                          nlohmann::json intrinsics, nlohmann::json extrinsics,
                          bool debug);
}
