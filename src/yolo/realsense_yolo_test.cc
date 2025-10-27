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
  std::cout << "Drawing " << boxes.size() << " boxes" << std::endl;
  for (size_t i = 0; i < boxes.size(); i++) {
    cv::Scalar color(0, 255, 0);
    cv::rectangle(img, boxes[i], color, 2);
    std::string label =
        class_names[class_ids[i]] + " " + cv::format("%.2f", confidences[i]);
    int baseline = 0;
    cv::Size label_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    cv::rectangle(
        img, cv::Point(boxes[i].x, boxes[i].y - label_size.height - baseline),
        cv::Point(boxes[i].x + label_size.width, boxes[i].y), color,
        cv::FILLED);
    cv::putText(img, label, cv::Point(boxes[i].x, boxes[i].y - baseline),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
}

std::vector<float> SoftmaxResults(yolo::Yolo& model, const cv::Mat& mat,
                                  std::vector<cv::Rect>& bboxes,
                                  std::vector<float>& confidences,
                                  std::vector<int>& class_ids) {
  // Store original image dimensions
  int orig_h = mat.rows;
  int orig_w = mat.cols;

  // Calculate letterbox parameters (must match model preprocessing)
  const int target_size = 640;
  float scale =
      std::min(target_size / (float)orig_h, target_size / (float)orig_w);

  int new_w = round(orig_w * scale);
  int new_h = round(orig_h * scale);

  float pad_left = (target_size - new_w) / 2.0;
  float pad_top = (target_size - new_h) / 2.0;

  std::vector<float> softmax_results = model.RunModel(mat);
  const int nms_output_size = 6;
  for (int i = 0; i < 10; i++) {
    float x1 = softmax_results[i * nms_output_size];
    float y1 = softmax_results[i * nms_output_size + 1];
    float x2 = softmax_results[i * nms_output_size + 2];
    float y2 = softmax_results[i * nms_output_size + 3];
    float confidence = softmax_results[i * nms_output_size + 4];
    int id = softmax_results[i * nms_output_size + 5];

    x1 = (x1 - pad_left) / scale;
    y1 = (y1 - pad_top) / scale;
    x2 = (x2 - pad_left) / scale;
    y2 = (y2 - pad_top) / scale;

    x1 = std::max(0.0f, std::min(x1, (float)orig_w));
    y1 = std::max(0.0f, std::min(y1, (float)orig_h));
    x2 = std::max(0.0f, std::min(x2, (float)orig_w));
    y2 = std::max(0.0f, std::min(y2, (float)orig_h));

    float bbox_width = x2 - x1;
    float bbox_height = y2 - y1;

    bboxes[i] = cv::Rect(x1, y1, bbox_width, bbox_height);
    confidences[i] = confidence;
    class_ids[i] = id;
  }
  return softmax_results;
}

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/model/fifthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath, true);
  camera::RealSenseCamera rs_camera;
  std::vector<cv::Rect> bboxes(6);
  std::vector<float> confidences(6);
  std::vector<int> class_ids(6);
  std::vector<float> softmax_results;
  std::vector<std::string> class_names = {"CORAL", "ALGAE"};
  while (true) {
    std::cout << 1 << std::endl;
    cv::Mat mat;
    rs_camera.getFrame(mat);
    if (mat.empty()) {
      std::cout << "Couldn't fetch frame properly" << std::endl;
      return 1;
    }
    std::cout << 2 << std::endl;
    softmax_results =
        SoftmaxResults(model, mat, bboxes, confidences, class_ids);
    std::cout << 3 << std::endl;
    drawDetections(mat, bboxes, class_ids, confidences, class_names);
    std::cout << 4 << std::endl;
    // cv::imshow("Test detections", mat);
    // cv::waitKey(0);
    // cv::destroyAllWindows();
  }
}
