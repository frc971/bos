#include "yolo.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "opencv2/cudawarping.hpp"

namespace yolo {
std::vector<char> loadEngineFile(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  if (!file)
    throw std::runtime_error("Engine file not found");
  return std::vector<char>((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
}

size_t getOutputSize(nvinfer1::ICudaEngine* engine) {
  nvinfer1::Dims output_shape =
      engine->getTensorShape(engine->getIOTensorName(1));
  size_t output_size = 1;
  for (int i = 0; i < output_shape.nbDims; i++) {
    output_size *= output_shape.d[i];
    printf("%ld ", output_shape.d[i]);
  }
  printf("\noutput size: %zu output nbDims: %d\n", output_size,
         output_shape.nbDims);

  return output_size;
}

void Yolo::preprocessImage(const cv::Mat& img, float* gpu_input,
                           const nvinfer1::Dims64& dims) {
  // cv::Mat rgb_image;
  // cv::cvtColor(img, rgb_image, cv::COLOR_BGR2RGB);
  cv::cuda::GpuMat img_gpu;
  img_gpu.upload(img);

  const int target_size = 640;  // new_shape
  const int channels = 3;

  // Compute scale factor to preserve aspect ratio
  int orig_h = img.rows;
  int orig_w = img.cols;
  float r = std::min(target_size / (float)orig_h, target_size / (float)orig_w);

  // Compute new unpadded size
  int new_w = int(round(orig_w * r));
  int new_h = int(round(orig_h * r));

  // Compute padding
  int dw = target_size - new_w;
  int dh = target_size - new_h;
  int top = int(round(dh / 2.0 - 0.1));
  int bottom = int(round(dh / 2.0 + 0.1));
  int left = int(round(dw / 2.0 - 0.1));
  int right = int(round(dw / 2.0 + 0.1));

  // Resize image to new unpadded size
  cv::cuda::GpuMat resized;
  cv::cuda::resize(img_gpu, resized, cv::Size(new_w, new_h), 0, 0,
                   cv::INTER_LINEAR);

  // Pad image to target size with gray color (114)
  cv::cuda::GpuMat padded;
  cv::Scalar color(114, 114, 114);
  cv::cuda::copyMakeBorder(resized, padded, top, bottom, left, right,
                           cv::BORDER_CONSTANT, color);

  // Normalize to 0-1
  cv::cuda::GpuMat normalized;
  padded.convertTo(normalized, CV_32FC3, 1.f / 255.f);

  // Prepare CHW layout directly in gpu_input
  int channel_size = target_size * target_size;
  std::vector<cv::cuda::GpuMat> chw;
  for (int i = 0; i < channels; i++) {
    chw.emplace_back(cv::cuda::GpuMat(cv::Size(target_size, target_size),
                                      CV_32FC1, gpu_input + i * channel_size));
  }
  cv::cuda::split(normalized, chw);
  cv::Mat cpu_mat;
  normalized.download(cpu_mat);
  cv::imshow("Modified image", cpu_mat);
  cv::waitKey(0);
}

class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING)
      std::printf("ERROR: %s", msg);
  }
};

Yolo::Yolo(std::string model_path, bool verbose) : verbose_(verbose) {
  Logger logger;
  std::vector<char> engine_data = loadEngineFile(model_path);

  runtime_ = nvinfer1::createInferRuntime(logger);
  assert(runtime_ != nullptr);

  engine_ =
      runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
  assert(engine_ != nullptr);

  context_ = engine_->createExecutionContext();
  assert(context_ != nullptr);

  const nvinfer1::Dims& input_dims_ =
      engine_->getTensorShape(engine_->getIOTensorName(0));
  size_t input_size = 1;
  for (int i = 0; i < input_dims_.nbDims; ++i) {
    input_size *= input_dims_.d[i];
  }

  cudaMalloc((void**)&input_buffer_, sizeof(float) * input_size);
  output_size_ = getOutputSize(engine_);
  cudaMalloc((void**)&(output_buffer_), sizeof(float) * output_size_);

  cudaStreamCreate(&(inferenceCudaStream_));
}

std::vector<float> Yolo::RunModel(const cv::Mat& frame) {
  bool status;
  preprocessImage(frame, input_buffer_, input_dims_);
  std::cout << "Preprocessed" << std::endl;
  status =
      context_->setTensorAddress(engine_->getIOTensorName(0), input_buffer_);
  assert(status);
  std::cout << "Tensor address set 0" << std::endl;
  status =
      context_->setTensorAddress(engine_->getIOTensorName(1), output_buffer_);
  assert(status);
  std::cout << "Tensor address set 1" << std::endl;
  status = context_->enqueueV3(inferenceCudaStream_);
  std::cout << "Enqueued" << std::endl;
  assert(status);

  cudaStreamSynchronize(inferenceCudaStream_);
  std::cout << "Stream synchronized" << std::endl;
  std::vector<float> featureVector;
  featureVector.resize(output_size_);
  std::cout << "Resized" << std::endl;
  cudaMemcpy(featureVector.data(), output_buffer_, output_size_ * sizeof(float),
             cudaMemcpyDeviceToHost);
  std::cout << "Memcpy" << std::endl;
  if (verbose_) {
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 7; j++) {
        printf("%f ", featureVector[i * 7 + j]);
      }
      printf("\n");
    }
  }
  return featureVector;
}

Yolo::~Yolo() {
  if (context_) {
    delete context_;
    context_ = nullptr;
  }

  if (engine_) {
    delete engine_;
    engine_ = nullptr;
  }

  if (runtime_) {
    delete runtime_;
    runtime_ = nullptr;
  }

  if (output_buffer_) {
    cudaFree(output_buffer_);
    output_buffer_ = nullptr;
  }

  if (inferenceCudaStream_) {
    cudaStreamDestroy(inferenceCudaStream_);
    inferenceCudaStream_ = nullptr;
  }
}

}  // namespace yolo
