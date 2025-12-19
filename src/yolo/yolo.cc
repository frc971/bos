#include "yolo.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/opencv.hpp>
#include <ostream>
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
  }
  return output_size;
}

void Yolo::PreprocessImage(const cv::Mat& img, float* gpu_input,
                           const nvinfer1::Dims64& dims) {
  cv::Mat maybe_rgb;
  if (color_) {
    cv::cvtColor(img, maybe_rgb, cv::COLOR_BGR2RGB);
  } else {
    maybe_rgb = img;
  }
  cv::cuda::GpuMat img_gpu;
  img_gpu.upload(maybe_rgb);

  // Compute scale factor to preserve aspect ratio
  int orig_h = img.rows;
  int orig_w = img.cols;
  float scale =
      std::min(TARGET_SIZE / (float)orig_h, TARGET_SIZE / (float)orig_w);

  // Compute new unpadded size
  int new_w = round(orig_w * scale);
  int new_h = round(orig_h * scale);

  // Compute padding
  int dw = TARGET_SIZE - new_w;
  int dh = TARGET_SIZE - new_h;
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
  padded.convertTo(normalized, color_ ? CV_32FC3 : CV_32FC1, 1.f / 255.f);

  int channel_size = TARGET_SIZE * TARGET_SIZE;

  if (color_) {
    const int channels = color_ ? 3 : 1;

    // Split into channels (HWC -> CHW)
    std::vector<cv::cuda::GpuMat> chw(channels);
    cv::cuda::split(normalized, chw);
    for (int i = 0; i < channels; i++) {
      cudaMemcpy(gpu_input + i * channel_size, chw[i].data,
                 channel_size * sizeof(float), cudaMemcpyDeviceToDevice);
    }
  }
}

class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING)
      std::printf("ERROR: %s", msg);
  }
};

Yolo::Yolo(std::string model_path, const bool color, const bool verbose)
    : color_(color), verbose_(verbose) {
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
  (void)status;
  PreprocessImage(frame, input_buffer_, input_dims_);
  status =
      context_->setTensorAddress(engine_->getIOTensorName(0), input_buffer_);
  assert(status);
  status =
      context_->setTensorAddress(engine_->getIOTensorName(1), output_buffer_);
  assert(status);
  status = context_->enqueueV3(inferenceCudaStream_);
  assert(status);

  cudaStreamSynchronize(inferenceCudaStream_);
  std::vector<float> featureVector;
  featureVector.resize(output_size_);
  cudaMemcpy(featureVector.data(), output_buffer_, output_size_ * sizeof(float),
             cudaMemcpyDeviceToHost);
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

  if (input_buffer_) {
    cudaFree(input_buffer_);
    input_buffer_ = nullptr;
  }

  if (inferenceCudaStream_) {
    cudaStreamDestroy(inferenceCudaStream_);
    inferenceCudaStream_ = nullptr;
  }
}

double Yolo::GetObjectAngle(double object_position, double fov,
                            int image_width) {
  double focal_length = image_width / std::tan(fov / 2.0);
  return std::atan2(object_position - (image_width / 2.0), focal_length);
}

double Yolo::GetObjectDistance(double object_pitch, double camera_height) {
  return camera_height / std::tan(object_pitch);
}

void Yolo::DrawDetections(cv::Mat& img, const std::vector<cv::Rect>& boxes,
                          const std::vector<int>& class_ids,
                          const std::vector<float>& confidences,
                          const std::vector<std::string>& class_names) {
  for (size_t i = 0; i < boxes.size(); i++) {
    cv::Scalar color(0, 255, 0);
    cv::rectangle(img, boxes[i], color, 2);
    std::string label =
        class_names[class_ids[i]] + " " + cv::format("%.2f", confidences[i]);
    int baseline = 0;
    cv::Size label_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    cv::rectangle(
        img,
        cv::Point(boxes[i].x,
                  std::max(0, boxes[i].y - label_size.height - baseline)),
        cv::Point(boxes[i].x + label_size.width, boxes[i].y), color,
        cv::FILLED);
    cv::putText(img, label, cv::Point(boxes[i].x, boxes[i].y - baseline),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
}

std::vector<float> Yolo::Postprocess(const int original_height,
                                     const int original_width,
                                     const std::vector<float>& results,
                                     std::vector<cv::Rect>& bboxes,
                                     std::vector<float>& confidences,
                                     std::vector<int>& class_ids) {
  float scale = std::min(TARGET_SIZE / (float)original_height,
                         TARGET_SIZE / (float)original_width);

  const int new_w = round(original_width * scale);
  const int new_h = round(original_height * scale);

  float pad_left = (TARGET_SIZE - new_w) / 2.0;
  float pad_top = (TARGET_SIZE - new_h) / 2.0;
  const int nms_output_size = 6;
  for (size_t i = 0; i < bboxes.size(); i++) {
    float x1 = results[i * nms_output_size];
    float y1 = results[i * nms_output_size + 1];
    float x2 = results[i * nms_output_size + 2];
    float y2 = results[i * nms_output_size + 3];
    float confidence = results[i * nms_output_size + 4];
    int id = results[i * nms_output_size + 5];

    x1 = (x1 - pad_left) / scale;
    y1 = (y1 - pad_top) / scale;
    x2 = (x2 - pad_left) / scale;
    y2 = (y2 - pad_top) / scale;

    x1 = std::max(0.0f, std::min(x1, (float)original_width));
    y1 = std::max(0.0f, std::min(y1, (float)original_height));
    x2 = std::max(0.0f, std::min(x2, (float)original_width));
    y2 = std::max(0.0f, std::min(y2, (float)original_height));

    float bbox_width = x2 - x1;
    float bbox_height = y2 - y1;

    bboxes[i] = cv::Rect(x1, y1, bbox_width, bbox_height);
    confidences[i] = confidence;
    class_ids[i] = id;
  }
  return results;
}

}  // namespace yolo
