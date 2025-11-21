#include <iostream>
#include <frc/geometry/Pose2d.h>
#include <opencv2/opencv.hpp>
#include "src/camera/realsense_camera.h"
#include "src/nt_utils.h"
#include "src/yolo/yolo.h"
#include <networktables/StructTopic.h>

int main() {
  std::cout << "Starting gamepiece main" << std::endl;
  std::cout << "Started networktables" << std::endl;
  yolo::Yolo model("/bos/src/yolo/model/ninthYOLO.engine");
  camera::RealSenseCamera rs;
  cv::Mat color;
  cv::Mat depth;
  constexpr int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {"algae", "algae", "coral", "coral"};
  NTUtils::start_networktables();
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> coral_table = inst.GetTable("Orin/Gamepiece/coral");
  std::shared_ptr<nt::NetworkTable> algae_table = inst.GetTable("Orin/Gamepiece/algae");
  nt::StructTopic<frc::Pose2d> coral_topic = coral_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructPublisher coral_pub = coral_topic.Publish();
  nt::StructTopic<frc::Pose2d> algae_topic = algae_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructPublisher algae_pub = algae_topic.Publish();

  while (true) {
    rs.getFrame(color, depth);
    if (color.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    model.Postprocess(color, bboxes, confidences, class_ids);
    for (size_t i = 0; i < max_detections; i++) {
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
          distance = depth.at<float>(c_y + j, c_x);
          if (distance == 0)
            distance = depth.at<float>(c_y - j, c_x);
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
