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
      const float distance = depth.at<float>(bboxes[i].y + bboxes[i].height / 2,
                                             bboxes[i].x + bboxes[i].width / 2);
      std::cout << "Detected a " << class_names[class_ids[i]] << " " << distance
                << " meters away" << std::endl;
    }
  }
}
