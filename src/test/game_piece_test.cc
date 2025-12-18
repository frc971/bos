#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/StructTopic.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/realsense_camera.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/yolo/yolo.h"
#include <chrono>
#include <iomanip>
#include <filesystem>

static constexpr int MAX_DETECTIONS = 6;
static std::vector<std::string> class_names = {
    "algae", "algae", "coral",
    "coral"};  // Chopped because I screwed up on the dataset, and technically the model outputs "CORAL", "coral", "ALGAE" or "algae"
static std::mutex mutex;

std::ostream& operator<<(std::ostream& os, const frc::Pose3d& p) {
    os << "Point(" << p.X().value() << ", " << p.Y().value() << ", " << p.Z().value() << ")" << "\nRotation:\nPitch:\t" << p.Rotation().Y().value() << "\nRoll:\t" << p.Rotation().X().value() << "\nYaw:\t" << p.Rotation().Z().value() << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const frc::Transform3d& p) {
    os << "Point(" << p.X().value() << ", " << p.Y().value() << ", " << p.Z().value() << ")" << "\nRotation:\nPitch:\t" << p.Rotation().Y().value() << "\nRoll:\t" << p.Rotation().X().value() << "\nYaw:\t" << p.Rotation().Z().value() << ")";
    return os;
}

void run_gamepiece_detect(yolo::Yolo& model,
                          cv::Mat frame,
                          nlohmann::json intrinsics,
                          nlohmann::json extrinsics) {
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  const float pinhole_height = extrinsics
      ["translation_z"];  // Height of the center of the camera, must be from GROUND not bumpers or smthn
  const float cam_cx = intrinsics["cx"];
  const float cam_cy = intrinsics["cy"];
  const float focal_length_vertical = intrinsics["fy"].get<float>() / 1000; // needs to be meters
  const float focal_length_horizontal = intrinsics["fx"].get<float>() / 1000;
  const float cam_pitch =
      -(float)extrinsics
          ["rotation_y"];  // negative because if the camera is tilted up, phi should be smaller than the theta read by camera
  const frc::Pose3d cam_pose{
      frc::Translation3d{
          units::meter_t{extrinsics["translation_x"].get<float>()},
          units::meter_t{extrinsics["translation_y"].get<float>()},
          units::meter_t{extrinsics["translation_z"].get<float>()}},
      frc::Rotation3d{
          units::radian_t{extrinsics["rotation_x"].get<float>()},
          units::radian_t{extrinsics["rotation_y"].get<float>()},
          units::radian_t{(float)extrinsics["rotation_z"].get<float>()}}};
  frc::Transform3d target_pose_cam_relative;
  frc::Pose3d target_pose_robot_relative;
  while (true) {
    mutex.lock();
    model.Postprocess(frame.rows, frame.cols, model.RunModel(frame), bboxes,
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
          atan2(c_y - cam_cy, focal_length_vertical);
      const float phi = cam_relative_pitch + cam_pitch;
      const float distance = pinhole_height / sin(phi);
      const std::string& class_name = class_names[class_ids[i]];
      std::cout << "\tc_y:\t" << c_y << "\tcam_relative_pitch:\t" << cam_relative_pitch << "\tphi:\t" << phi << std::endl;
      std::cout << "Detected a " << class_name << " " << distance
                << " meters away" << std::endl;
      const float cam_relative_yaw =
          atan2(c_x - cam_cx, focal_length_horizontal);
      target_pose_cam_relative = {
          frc::Translation3d{
              units::meter_t{distance * cos(cam_relative_pitch) *
                             cos(cam_relative_yaw)},
              units::meter_t{distance * cos(cam_relative_pitch) *
                             sin(cam_relative_yaw)},
              units::meter_t{distance * sin(cam_relative_pitch)}},
          frc::Rotation3d{0_rad, units::radian_t{cam_relative_pitch},
                          units::radian_t{-cam_relative_yaw}}};
      std::cout << "TargetPose: " << target_pose_cam_relative << std::endl;
      target_pose_robot_relative =
          cam_pose.TransformBy(target_pose_cam_relative);
      if (class_name == "coral") {
        model.DrawDetections(frame, bboxes, class_ids, confidences, class_names);
        cv::imwrite(std::string(std::getenv("HOME")) + "/Documents/tested/" +
                        "redo.png",
                    frame);
        std::exit(0);
      }
    }
  }
}

int main() {
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  yolo::Yolo color_model("/bos/models/color.engine", true);
  for (const auto& entry : std::filesystem::directory_iterator(std::string(std::getenv("HOME")) + "/Documents/collected_imgs")) {
    cv::Mat mat = cv::imread(entry.path().string());
    run_gamepiece_detect(color_model, mat, utils::read_intrinsics("/bos/constants/realsense_intrinsics.json"),
        utils::read_extrinsics("/bos/constants/realsense_extrinsics.json"));
  }
}
