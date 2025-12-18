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
                          std::unique_ptr<camera::ICamera> camera,
                          nt::StructTopic<frc::Pose2d>& coral_topic,
                          nt::StructTopic<frc::Pose2d>& algae_topic,
                          nlohmann::json intrinsics,
                          nlohmann::json extrinsics) {
  nt::StructPublisher<frc::Pose2d> coral_pub = coral_topic.Publish();
  nt::StructPublisher<frc::Pose2d> algae_pub = algae_topic.Publish();
  cv::Mat color;
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
    camera->GetFrame(color);
    mutex.lock();
    model.Postprocess(color.rows, color.cols, model.RunModel(color), bboxes,
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
        coral_pub.Set(target_pose_robot_relative.ToPose2d());
        model.DrawDetections(color, bboxes, class_ids, confidences, class_names);
        cv::imwrite(std::string(std::getenv("HOME")) + "/Documents/tested/" +
                        "frame.png",
                    color);
        std::exit(0);
      } else {
        algae_pub.Set(target_pose_robot_relative.ToPose2d());
      }
    }
  }
}

void run_gamepiece_detect_realsense(yolo::Yolo& model,
                                    std::unique_ptr<camera::RealSenseCamera> rs,
                                    nt::StructTopic<frc::Pose2d>& coral_topic,
                                    nt::StructTopic<frc::Pose2d>& algae_topic,
                                    nlohmann::json intrinsics,
                                    nlohmann::json extrinsics) {
  nt::StructPublisher<frc::Pose2d> coral_pub = coral_topic.Publish();
  nt::StructPublisher<frc::Pose2d> algae_pub = algae_topic.Publish();
  cv::Mat color;
  cv::Mat depth;
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  const frc::Pose3d cam_pose{
      frc::Translation3d{
          units::meter_t{extrinsics["translation_x"].get<float>()},
          units::meter_t{extrinsics["translation_y"].get<float>()},
          units::meter_t{extrinsics["translation_z"].get<float>()}},
      frc::Rotation3d{
          units::radian_t{extrinsics["rotation_x"].get<float>()},
          units::radian_t{extrinsics["rotation_y"].get<float>()},
          units::radian_t{(float)extrinsics["rotation_z"].get<float>()}}};
  const float cam_cx = intrinsics["cx"];
  const float cam_cy = intrinsics["cy"];
  const float focal_length_horizontal = intrinsics["fx"];
  frc::Transform3d target_pose_cam_relative;
  const float focal_length_vertical = intrinsics["fy"];
  frc::Pose3d target_pose_robot_relative;
  while (true) {
    rs->GetFrame(color, depth);
    mutex.lock();
    model.Postprocess(color.rows, color.cols, model.RunModel(color), bboxes,
                      confidences, class_ids);
    mutex.unlock();
    if (color.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      continue;
    }
    for (size_t i = 0; i < MAX_DETECTIONS; i++) {
      if (bboxes[i].empty()) {
        if (i == 0) {
          std::cout << "No detections" << std::endl;
        }
        break;
      }
      const int c_y = bboxes[i].y + bboxes[i].height / 2;
      const int c_x = bboxes[i].x + bboxes[i].width / 2;
      float distance = depth.at<float>(c_y, c_x);
      if (distance == 0) {
        for (int j = 0; distance == 0 && j < bboxes[i].width / 2;
             j += bboxes[i].width / 10) {
          distance = depth.at<float>(c_y, c_x + j);
          if (distance == 0)
            distance = depth.at<float>(c_y, c_x - j);
        }
        for (int j = 0; distance == 0 && j < bboxes[i].height / 2;
             j += bboxes[i].height / 10) {
          if (distance == 0) {
            distance = depth.at<float>(c_y + j, c_x);
            distance = depth.at<float>(c_y - j, c_x);
          }
        }
        if (distance == 0) {
          std::cout << "Couldn't find reasonable distance value after scan, "
                       "discarding this one"
                    << std::endl;
          continue;
        }
      }

      const std::string& class_name = class_names[class_ids[i]];

      std::cout << "Detected a " << class_name << " " << distance
                << " meters away" << std::endl;

      const float cam_relative_pitch =
          atan2(c_y - cam_cy, focal_length_vertical);
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
      target_pose_robot_relative =
          cam_pose.TransformBy(target_pose_cam_relative);

      if (class_name == "coral") {
        coral_pub.Set(target_pose_robot_relative.ToPose2d());
      } else {
        algae_pub.Set(target_pose_robot_relative.ToPose2d());
      }
    }
  }
}

int main() {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Starting gamepiece main" << std::endl;
  std::cout << "Started networktables" << std::endl;
  yolo::Yolo color_model("/bos/models/color.engine", true);
  // yolo::Yolo gray_model("/bos/models/gray.engine", false);
  utils::StartNetworktables();
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> coral_table =
      inst.GetTable("Orin/Gamepiece/coral");
  std::shared_ptr<nt::NetworkTable> algae_table =
      inst.GetTable("Orin/Gamepiece/algae");
  nt::StructTopic<frc::Pose2d> coral_topic =
      coral_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose2d> algae_topic =
      algae_table->GetStructTopic<frc::Pose2d>("Pose");

  std::vector<std::thread> camera_threads;
  const bool using_rs = false;
  if (using_rs) {
    // purely for testing bc mechanical is mean to realsense :(
    camera_threads.emplace_back(
        run_gamepiece_detect_realsense, std::ref(color_model),
        std::make_unique<camera::RealSenseCamera>(), std::ref(coral_topic),
        std::ref(algae_topic),
        utils::read_intrinsics(
            camera::camera_constants[camera::Camera::USB0].intrinsics_path),
        utils::read_extrinsics(
            camera::camera_constants[camera::Camera::USB0].extrinsics_path));
  } else {
    camera_threads.emplace_back(
        run_gamepiece_detect, std::ref(color_model),
        std::make_unique<camera::RealSenseCamera>(), std::ref(coral_topic),
        std::ref(algae_topic),
        utils::read_intrinsics("/bos/constants/realsense_intrinsics.json"),
        utils::read_extrinsics("/bos/constants/realsense_extrinsics.json"));
  }
  camera_threads[0].join();
}
