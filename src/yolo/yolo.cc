#include "yolo.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

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

void preprocessImage(const cv::Mat& img, float* gpu_input,
                     const nvinfer1::Dims& dims) {
  std::cout << "Entering preprocessing" << std::endl;
  cv::cuda::GpuMat img_gpu;
  img_gpu.upload(img);
  const int channels = dims.d[0];
  const int height = dims.d[1];
  const int width = dims.d[2];
  cv::cuda::GpuMat resized;
  cv::resize(img_gpu, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
  cv::cuda::GpuMat normalized;
  resized.convertTo(normalized, CV_32FC3, 1.f / 255.f);
  const int channel_width = width * height;
  std::vector<cv::cuda::GpuMat> chw;
  chw.reserve(channels);
  for (int i = 0; i < channels; i++) {
    chw.emplace_back(cv::cuda::GpuMat(cv::Size(width, height), CV_32FC3,
                                      gpu_input + i * channel_width));
  }
  cv::cuda::split(normalized, chw);
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

std::vector<float> Yolo::RunModel(cv::Mat frame) {
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
