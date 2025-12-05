// https://gist.github.com/SteveRuben/1a15909e384b582c51b5
#pragma once
#include <NvInfer.h>
#include <stdio.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
namespace yolo {

class Yolo {
 public:
  Yolo(std::string model_path, bool color, bool verbose = false);
  ~Yolo();
  std::vector<float> RunModel(const cv::Mat& frame);
  std::vector<float> Postprocess(const int original_height, const int original_width, const std::vector<float>& results,
                                 std::vector<cv::Rect>& bboxes,
                                 std::vector<float>& confidences,
                                 std::vector<int>& class_ids);
  static void DrawDetections(cv::Mat& img, const std::vector<cv::Rect>& boxes,
                             const std::vector<int>& class_ids,
                             const std::vector<float>& confidences,
                             const std::vector<std::string>& class_names);
  static constexpr int TARGET_SIZE = 640;

 private:
  void PreprocessImage(const cv::Mat& frame, float* gpu_input,
                              const nvinfer1::Dims64& dims);
  nvinfer1::IRuntime* runtime_;
  nvinfer1::ICudaEngine* engine_;
  nvinfer1::IExecutionContext* context_;
  cudaStream_t inferenceCudaStream_;
  nvinfer1::Dims64 input_dims_;
  const bool color_;
  float* input_buffer_;
  float* output_buffer_;
  size_t output_size_;
  bool verbose_;
};

}  // namespace yolo
