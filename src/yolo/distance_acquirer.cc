#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/realsense_camera.h"
#include "src/yolo/yolo.h"

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/ninthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath);
  camera::RealSenseCamera rs_camera;
  const int max_detections = 6;
  std::vector<cv::Rect> bboxes(max_detections);
  std::vector<float> confidences(max_detections);
  std::vector<int> class_ids(max_detections);
  std::vector<std::string> class_names = {"Algae", "ALGAE", "Coral", "CORAL"};
  while (true) {
    cv::Mat color_mat;
    cv::Mat depth_mat;
    rs_camera.getFrame(color_mat, depth_mat);
    if (color_mat.empty() || depth_mat.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    model.Postprocess(color_mat, bboxes, confidences, class_ids);
    yolo::Yolo::DrawDetections(color_mat, bboxes, class_ids, confidences,
                               class_names);
    const int c_x = bboxes[0].x + bboxes[0].width / 2;
    const int c_y = bboxes[0].y + bboxes[0].height / 2;

    std::string depth_label =
        "Depth " + std::to_string(depth_mat.at<float>(c_x, c_y));
    std::cout << "Considering pixel: (" << c_x << ", " << c_y << ")"
              << std::endl;
    cv::Size label_size =
        cv::getTextSize(depth_label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
    cv::putText(
        color_mat, depth_label,
        cv::Point(c_x - label_size.width / 2, c_y + label_size.height / 2),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    cv::imshow("Test detections", color_mat);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}
