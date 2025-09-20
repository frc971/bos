// https://gist.github.com/SteveRuben/1a15909e384b582c51b5
#ifndef YOLO_H
#define YOLO_H
#include <NvInfer.h>
#include <stdio.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
namespace Yolo {

class Yolo {
 public:
  Yolo(std::string model_path, bool verbose = false);
  ~Yolo();
  std::vector<float> RunModel(cv::Mat frame);

 private:
  nvinfer1::IRuntime* runtime_;
  nvinfer1::ICudaEngine* engine_;
  nvinfer1::IExecutionContext* context_;
  cudaStream_t inferenceCudaStream_;
  void* output_buffer_;
  size_t output_size_;
  bool verbose_;
};

}  // namespace Yolo

#endif  // YOLO_H
