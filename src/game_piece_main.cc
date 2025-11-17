#include <iostream>
#include <opencv2/opencv.hpp>
#include "src/camera/realsense_camera.h"
#include "src/nt_utils.h"
#include "src/yolo/yolo.h"

int main() {
  std::cout << "Starting gamepiece main" << std::endl;
  // NTUtils::start_networktables();
  std::cout << "Started networktables" << std::endl;
  yolo::Yolo model("/bos/src/yolo/model/ninthYOLO.engine");
  camera::RealSenseCamera rs;
  cv::Mat color;
  cv::Mat depth;
  constexpr int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {"Algae", "ALGAE", "Coral", "CORAL"};
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

      std::cout << "Detected a " << class_names[class_ids[i]] << " " << distance
                << " meters away" << std::endl;
    }
  }
}
