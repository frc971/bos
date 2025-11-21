#include <iostream>
#include <frc/geometry/Pose2d.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/camera.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/realsense_camera.h"
#include "src/nt_utils.h"
#include "src/yolo/yolo.h"
#include "src/camera/camera_constants.h"
#include <networktables/StructTopic.h>

static constexpr int MAX_DETECTIONS = 6;

void run_gamepiece_detect(yolo::Yolo& model, std::unique_ptr<camera::CVCamera> camera, nt::StructTopic<frc::Pose2d>& coral_topic, nt::StructTopic<frc::Pose2d>& algae_topic/*, nlohmann::json intrinsics, nlohmann::json extrinsics*/) {
  
}

void run_gamepiece_detect_w_ir(yolo::Yolo& model, std::unique_ptr<camera::RealSenseCamera> rs, nt::StructPublisher<frc::Pose2d> coral_pub, nt::StructPublisher<frc::Pose2d> algae_pub/*, nlohmann::json intrinsics, nlohmann::json extrinsics*/) {
  cv::Mat color;
  cv::Mat depth;
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  std::vector<std::string> class_names = {"algae", "algae", "coral", "coral"};
  while (true) {
    rs->GetFrame(color, depth);
    if (color.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      continue;
    }
    for (size_t i = 0; i < MAX_DETECTIONS; i++) {
      model.Postprocess(color, bboxes, confidences, class_ids);
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

      const double target_angle = std::asin(c_x / (double)distance);
      const frc::Pose2d target_pose(units::meter_t{distance * cos(target_angle)}, units::meter_t{distance * sin(target_angle)}, units::radian_t{target_angle});
      
      if (class_name == "coral") {
        coral_pub.Set(target_pose);
      }
      else {
          algae_pub.Set(target_pose);
      }
    }
  }
}

int main() {
  std::cout << "Starting gamepiece main" << std::endl;
  std::cout << "Started networktables" << std::endl;
  yolo::Yolo model("/bos/src/yolo/model/ninthYOLO.engine");
  camera::RealSenseCamera rs;
  NTUtils::start_networktables();
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> coral_table = inst.GetTable("Orin/Gamepiece/coral");
  std::shared_ptr<nt::NetworkTable> algae_table = inst.GetTable("Orin/Gamepiece/algae");
  nt::StructTopic<frc::Pose2d> coral_topic = coral_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose2d> algae_topic = algae_table->GetStructTopic<frc::Pose2d>("Pose");

  std::vector<std::thread> camera_threads;
  const bool using_rs = true;
  if (using_rs) {
    // camera_threads.emplace_back(run_gamepiece_detect_w_ir, std::ref(model), std::make_unique<camera::RealSenseCamera>(), coral_pub, algae_pub);
  }
  else {
    camera_threads.reserve(2);
    std::unique_ptr<camera::CVCamera> usb0 = std::make_unique<camera::CVCamera>(cv::VideoCapture(camera::camera_constants[camera::Camera::USB0].pipeline));
std::unique_ptr<camera::CVCamera> usb1 = std::make_unique<camera::CVCamera>(cv::VideoCapture(camera::camera_constants[camera::Camera::USB0].pipeline));
    camera_threads.emplace_back(run_gamepiece_detect, std::ref(model), std::move(usb0), std::ref(coral_topic), std::ref(algae_topic));
    // camera_threads.emplace_back(run_gamepiece_detect, std::ref(model), usb1, coral_pub, algae_pub);

  }
}
