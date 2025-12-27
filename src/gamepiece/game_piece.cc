#include "src/gamepiece/game_piece.h"
#include <frc/geometry/Pose2d.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/realsense_camera.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

namespace gamepiece {
static constexpr int MAX_DETECTIONS = 6;
static std::mutex mutex;

std::ostream& operator<<(std::ostream& os, const frc::Pose3d& p) {
  os << "Point(" << p.X().value() << ", " << p.Y().value() << ", "
     << p.Z().value() << ")"
     << "\nRotation:\nPitch:\t" << p.Rotation().Y().value() << "\nRoll:\t"
     << p.Rotation().X().value() << "\nYaw:\t" << p.Rotation().Z().value()
     << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const frc::Transform3d& p) {
  os << "Point(" << p.X().value() << ", " << p.Y().value() << ", "
     << p.Z().value() << ")"
     << "\nRotation:\nPitch:\t" << p.Rotation().Y().value() << "\nRoll:\t"
     << p.Rotation().X().value() << "\nYaw:\t" << p.Rotation().Z().value()
     << ")";
  return os;
}

GamepieceDetector::GamepieceDetector(yolo::Yolo& model,
                                     yolo::ModelInfo& model_info,
                          const std::shared_ptr<camera::CameraSource> camera,
                          nt::StructTopic<frc::Pose2d>& coral_topic,
                          nt::StructTopic<frc::Pose2d>& algae_topic,
                          nlohmann::json intrinsics, nlohmann::json extrinsics): model_(model), camera_(camera), coral_topic_(coral_topic), algae_topic_(algae_topic), pinhole_height_(extrinsics
      ["translation_z"]), cam_cx_(intrinsics["cx"]), cam_cy_(intrinsics["cy"]), focal_length_vertical_(intrinsics["fy"]), focal_length_horizontal_(intrinsics["fx"]), cam_pitch_(extrinsics["rotation_y"]), cam_pose_{frc::Translation3d{
          units::meter_t{extrinsics["translation_x"].get<float>()},
          units::meter_t{extrinsics["translation_y"].get<float>()},
          units::meter_t{extrinsics["translation_z"].get<float>()}},
      frc::Rotation3d{
          units::radian_t{extrinsics["rotation_x"].get<float>()},
          units::radian_t{extrinsics["rotation_y"].get<float>()},
          units::radian_t{extrinsics["rotation_z"].get<float>()}}}, class_names_(model_info.class_names) {
}

void GamepieceDetector::run_gamepiece_detect(bool debug) {
  camera::CscoreStreamer streamer(camera_->GetName(), 4973, 30, 1080, 1080);
  nt::StructPublisher<frc::Pose2d> coral_pub = coral_topic_.Publish();
  nt::StructPublisher<frc::Pose2d> algae_pub = algae_topic_.Publish();
  cv::Mat color;
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  frc::Transform3d target_pose_cam_relative;
  frc::Pose3d target_pose_robot_relative;
  while (true) {
    camera::timestamped_frame_t timestamped_frame = camera_->Get();
    color = timestamped_frame.frame;
    mutex.lock();
    model_.Postprocess(color.rows, color.cols, model_.RunModel(color), bboxes,
                      confidences, class_ids);
    mutex.unlock();
    for (size_t i = 0; i < MAX_DETECTIONS; i++) {
      if (bboxes[i].empty()) {
        if (i == 0) {
          std::cout << "No detections" << std::endl;
        }
        break;
      }
      const int c_y = bboxes[i].y + bboxes[i].height / 2;
      const int c_x = bboxes[i].x + bboxes[i].width / 2;
      const float cam_relative_pitch =
          atan2(c_y - cam_cy_, focal_length_vertical_);
      const float phi = cam_relative_pitch + cam_pitch_;
      const float distance = pinhole_height_ / sin(phi);
      const std::string& class_name = class_names_[class_ids[i]];
      const float cam_relative_yaw =
          atan2(c_x - cam_cx_, focal_length_horizontal_);
      target_pose_cam_relative = {
          frc::Translation3d{
              units::meter_t{distance * cos(cam_relative_pitch) *
                             cos(cam_relative_yaw)},
              units::meter_t{distance * cos(cam_relative_pitch) *
                             sin(cam_relative_yaw)},
              units::meter_t{distance * -sin(cam_relative_pitch)}},
          frc::Rotation3d{0_rad, units::radian_t{cam_relative_pitch},
                          units::radian_t{-cam_relative_yaw}}};
      target_pose_robot_relative =
          cam_pose_.TransformBy(target_pose_cam_relative);
      if (class_name == "coral") {
        coral_pub.Set(target_pose_robot_relative.ToPose2d());
      } else {
        algae_pub.Set(target_pose_robot_relative.ToPose2d());
      }
      if (debug) {
        std::cout << "\tc_y:\t" << c_y << "\tcam_relative_pitch:\t"
                  << cam_relative_pitch << "\tphi:\t" << phi << "\tyaw:\t"
                  << cam_relative_yaw << std::endl;
        std::cout << "Detected a " << class_name << " " << distance
                  << " meters away" << std::endl;
        std::cout << "TargetPose: " << target_pose_cam_relative << std::endl;
        std::cout << "Robot_relative: " << target_pose_robot_relative
                  << std::endl;
        yolo::Yolo::DrawDetections(timestamped_frame.frame, bboxes, class_ids,
                               confidences, class_names_);
        streamer.WriteFrame(timestamped_frame.frame);
      }
    }
  }
}
} // gamepiece
