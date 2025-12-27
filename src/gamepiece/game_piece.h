#pragma once
#include "src/yolo/yolo.h"
#include <networktables/StructTopic.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include "src/camera/camera_source.h"
#include <nlohmann/json.hpp>
#include "src/yolo/yolo.h"
#include "src/yolo/model_constants.h"

namespace gamepiece {
class GamepieceDetector {
public:
  GamepieceDetector(yolo::Yolo& model,
                    yolo::ModelInfo& model_info,
                          const std::shared_ptr<camera::CameraSource> camera,
                          nt::StructTopic<frc::Pose2d>& coral_topic,
                          nt::StructTopic<frc::Pose2d>& algae_topic,
                          nlohmann::json intrinsics, nlohmann::json extrinsics);
  void run_gamepiece_detect(bool debug);

private:
  yolo::Yolo& model_;
  const std::shared_ptr<camera::CameraSource> camera_;
  nt::StructTopic<frc::Pose2d>& coral_topic_;
  nt::StructTopic<frc::Pose2d>& algae_topic_;
  const float pinhole_height_;
  const float cam_cx_;
  const float cam_cy_;
  const float focal_length_vertical_;
  const float focal_length_horizontal_;
  const float cam_pitch_;
  const frc::Pose3d cam_pose_;
  const std::vector<std::string> class_names_;
};
} // gamepiece
