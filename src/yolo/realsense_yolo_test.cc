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
  const int target_size = 640;  // new_shape
  const int channels = 3;

  // Compute scale factor to preserve aspect ratio
  int orig_h = img.rows;
  int orig_w = img.cols;
  float scale =
      std::min(target_size / (float)orig_h, target_size / (float)orig_w);

  // Compute new unpadded size
  int new_w = round(orig_w * scale);
  int new_h = round(orig_h * scale);
  std::cout << "O_H: " << orig_h << " O_W: " << orig_w << std::endl;
  std::cout << "N_H: " << new_h << " N_W: " << new_w << std::endl;

  // Compute padding
  int dw = target_size - new_w;
  int dh = target_size - new_h;
  int top = int(round(dh / 2.0 - 0.1));
  int bottom = int(round(dh / 2.0 + 0.1));
  int left = int(round(dw / 2.0 - 0.1));
  int right = int(round(dw / 2.0 + 0.1));

  // Resize image to new unpadded size
  cv::resize(img, img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  // Pad image to target size with gray color (114)
  cv::Scalar color(114, 114, 114);
  cv::copyMakeBorder(img, img, top, bottom, left, right, cv::BORDER_CONSTANT,
                     color);

  // Normalize to 0-1
  img.convertTo(img, CV_32FC3, 1.f / 255.f);

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
    printf("CenterX: %f, CenterY: %f,w: %f,h: %f,confidence: %f,id: %d", c_x,
           c_y, w, h, confidence, id);
    bboxes[i] = cv::Rect(c_x - w / 2, c_y - h / 2, c_x + w / 2, c_y + h / 2);
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
      "20250122_110501_mp4-0057_jpg.rf.ae930171a2c38361d57e04db5d1e07f3.jpg");
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
