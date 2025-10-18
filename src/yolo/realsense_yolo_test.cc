#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/realsense_camera.h"
#include "src/yolo/yolo.h"

static void drawDetections(cv::Mat& img, const std::vector<cv::Rect>& boxes,
                           const std::vector<int>& class_ids,
                           const std::vector<float>& confidences,
                           const std::vector<std::string>& class_names) {
  std::cout << "Drawing" << std::endl;
  for (size_t i = 0; i < boxes.size(); i++) {
    std::cout << "Run" << i << std::endl;
    cv::Scalar color(0, 255, 0);
    cv::rectangle(img, boxes[i], color, 2);
    std::cout << "Rectangled" << std::endl;
    for (const int id : class_ids) {
      std::cout << id << std::endl;
    }
    std::string label =
        class_names[class_ids[i]] + " " + cv::format("%.2f", confidences[i]);
    std::cout << "Labeled" << std::endl;
    int baseline = 0;
    cv::Size label_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    std::cout << "Label sized" << std::endl;

    cv::rectangle(
        img, cv::Point(boxes[i].x, boxes[i].y - label_size.height - baseline),
        cv::Point(boxes[i].x + label_size.width, boxes[i].y), color,
        cv::FILLED);
    std::cout << "2nd Rectangle made" << std::endl;
    cv::putText(img, label, cv::Point(boxes[i].x, boxes[i].y - baseline),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    std::cout << "Text placed" << std::endl;
  }
}

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/fifthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath, true);
  camera::RealSenseCamera rs_camera;
  cv::Mat mat;
  cv::Mat displayFrame;
  rs_camera.getFrame(mat);
  if (mat.empty()) {
    std::cout << "Couldn't fetch frame properly" << std::endl;
    return 1;
  }
  displayFrame = mat.clone();
  const std::vector<float> maybe_softmax_results = model.RunModel(mat);
  std::cout << "Detection size: " << maybe_softmax_results.size() << std::endl;
  std::vector<cv::Rect> bboxes(6);
  std::vector<float> confidences(6);
  std::vector<int> class_ids(6);
  const int nms_output_size = 7;
  for (int i = 0; i < 6; i++) {
    std::cout << "Run " << i << std::endl;
    float c_x = maybe_softmax_results[i * nms_output_size];
    float c_y = maybe_softmax_results[i * nms_output_size + 1];
    float w = maybe_softmax_results[i * nms_output_size + 2];
    float h = maybe_softmax_results[i * nms_output_size + 3];
    float confidence = maybe_softmax_results[i * nms_output_size + 4];
    int id = maybe_softmax_results[i * nms_output_size + 5];
    printf("CenterX: %f, CenterY: %f,w: %f,h: %f,confidence: %f,id: %d", c_x,
           c_y, w, h, confidence, id);
    bboxes[i] = cv::Rect(c_x - w / 2, c_y - h / 2, w, h);
    confidences[i] = confidence;
    class_ids[i] = id;
  }
  std::vector<std::string> class_names = {"ALGAE", "CORAL"};
  drawDetections(mat, bboxes, class_ids, confidences, class_names);
  cv::imshow("Test detections", mat);
  cv::waitKey(0);
  cv::imshow("Original image", displayFrame);
  cv::waitKey(0);
  std::string filename = "output_image.png";

  bool success = cv::imwrite(filename, mat);

  if (success) {
    std::cout << "Image saved successfully to " << filename << std::endl;
  } else {
    std::cerr << "Error: Could not save image to " << filename << std::endl;
  }
}
