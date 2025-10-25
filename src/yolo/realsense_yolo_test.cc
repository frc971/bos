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

  for (size_t i = 0; i < boxes.size(); i++) {
    cv::Scalar color(0, 255, 0);
    cv::rectangle(img, boxes[i], color, 2);
    std::cout << "CLass id: " << class_ids[i] << " on run " << i << std::endl;
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

std::vector<float> SoftmaxResults(yolo::Yolo& model, cv::Mat mat,
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
  std::cout << "Detection size: " << softmax_results.size() << std::endl;
  const int nms_output_size = 6;
  for (int i = 0; i < 10; i++) {
    std::cout << "Run " << i << std::endl;
    float c_x = softmax_results[i * nms_output_size];
    float c_y = softmax_results[i * nms_output_size + 1];
    float w = softmax_results[i * nms_output_size + 2];
    float h = softmax_results[i * nms_output_size + 3];
    float confidence = softmax_results[i * nms_output_size + 4];
    int id = softmax_results[i * nms_output_size + 5];
    printf("CenterX: %f, CenterY: %f,w: %f,h: %f,confidence: %f,id: %d\n", c_x,
           c_y, w, h, confidence, id);
    float x1_letterbox = c_x - w / 2.0;
    float y1_letterbox = c_y - h / 2.0;
    float x2_letterbox = c_x + w / 2.0;
    float y2_letterbox = c_y + h / 2.0;

    std::cout << "x1letter: " << x1_letterbox
              << " y1_letterbox: " << y1_letterbox
              << " x2_letterbox: " << x2_letterbox
              << " y2_letterbox: " << y2_letterbox << std::endl;

    float x1_orig = (x1_letterbox - pad_left) / scale;
    float y1_orig = (y1_letterbox - pad_top) / scale;
    float x2_orig = (x2_letterbox - pad_left) / scale;
    float y2_orig = (y2_letterbox - pad_top) / scale;

    x1_orig = std::max(0.0f, std::min(x1_orig, (float)orig_w));
    y1_orig = std::max(0.0f, std::min(y1_orig, (float)orig_h));
    x2_orig = std::max(0.0f, std::min(x2_orig, (float)orig_w));
    y2_orig = std::max(0.0f, std::min(y2_orig, (float)orig_h));

    std::cout << "x1orig: " << x1_orig << " y1orig: " << y1_orig
              << " x2orig: " << x2_orig << " y2orig: " << y2_orig << std::endl;

    float bbox_w = x2_orig - x1_orig;
    float bbox_h = y2_orig - y1_orig;

    bboxes[i] = cv::Rect(x1_orig, y1_orig, bbox_w, bbox_h);
    confidences[i] = confidence;
    class_ids[i] = id;
  }
  return softmax_results;
}

int main() {
  std::filesystem::path modelPath = "/bos/src/yolo/fifthYOLO.engine";
  std::cout << "Importing model from " << modelPath << std::endl;
  std::cout << "File actually exists: " << std::filesystem::exists(modelPath)
            << std::endl;
  yolo::Yolo model(modelPath, true);
  camera::RealSenseCamera rs_camera;
  cv::Mat mat;
  rs_camera.getFrame(mat);
  if (mat.empty()) {
    std::cout << "Couldn't fetch frame properly" << std::endl;
    return 1;
  }
  std::vector<cv::Rect> bboxes(6);
  std::vector<float> confidences(6);
  std::vector<int> class_ids(6);
  std::vector<float> softmax_results =
      SoftmaxResults(model, mat, bboxes, confidences, class_ids);
  std::vector<std::string> class_names = {"ALGAE", "CORAL"};
  drawDetections(mat, bboxes, class_ids, confidences, class_names);
  cv::imshow("Test detections", mat);
  cv::waitKey(0);
  std::string filename = "output_image.png";
  cv::Mat train_img = cv::imread(
      "/home/nvidia/Documents/gamepiece-data/test/images/"
      "20250122_101406_jpg.rf.0eacf8c2b7e1e10ea6520ff58ccba153.jpg");
  softmax_results =
      SoftmaxResults(model, train_img, bboxes, confidences, class_ids);
  drawDetections(train_img, bboxes, class_ids, confidences, class_names);
  cv::imshow("TrainImg", train_img);
  cv::waitKey(0);

  bool success = cv::imwrite(filename, mat);

  if (success) {
    std::cout << "Image saved successfully to " << filename << std::endl;
  } else {
    std::cerr << "Error: Could not save image to " << filename << std::endl;
  }
}
