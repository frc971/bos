#include "yolo.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <vector>

namespace Yolo {
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

cv::cuda::GpuMat preprocess(cv::Mat& img) {
  cv::cuda::GpuMat img_gpu;
  img_gpu.upload(img);
  cv::cuda::GpuMat gpu_dst(1, img_gpu.rows * img_gpu.cols * 1, CV_8UC3);
  size_t width = img_gpu.cols * img_gpu.rows;
  std::vector<cv::cuda::GpuMat> input_channels{
      cv::cuda::GpuMat(img_gpu.rows, img_gpu.cols, CV_8U, &(gpu_dst.ptr()[0])),
      cv::cuda::GpuMat(img_gpu.rows, img_gpu.cols, CV_8U,
                       &(gpu_dst.ptr()[width])),
      cv::cuda::GpuMat(img_gpu.rows, img_gpu.cols, CV_8U,
                       &(gpu_dst.ptr()[width * 2]))};
  cv::cuda::split(img_gpu, input_channels);  // HWC -> CHW
  cv::cuda::GpuMat output;
  gpu_dst.convertTo(output, CV_32FC3, 1.f / 255.f);
  return output;
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

  output_size_ = getOutputSize(engine_);
  cudaMalloc((void**)&(output_buffer_), sizeof(float) * output_size_);

  cudaStreamCreate(&(inferenceCudaStream_));
}

std::vector<float> Yolo::RunModel(cv::Mat frame) {
  bool status;
  cv::cuda::GpuMat gpu_mat = preprocess(frame);
  status = context_->setTensorAddress(engine_->getIOTensorName(0),
                                      (void*)gpu_mat.ptr<void>());
  assert(status);
  status = context_->setTensorAddress(engine_->getIOTensorName(1),
                                      (void*)output_buffer_);
  assert(status);
  status = context_->enqueueV3(inferenceCudaStream_);
  assert(status);

  cudaStreamSynchronize(inferenceCudaStream_);
  std::vector<float> featureVector;
  featureVector.resize(output_size_);
  cudaMemcpy(featureVector.data(), static_cast<char*>(output_buffer_),
             output_size_ * sizeof(float),
             cudaMemcpyDeviceToHost);  // Probably do not need to cast to char*

  if (verbose_) {
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 6; j++) {
        printf("%f ", featureVector[i * 6 + j]);
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

}  // namespace Yolo
