#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
namespace camera {
using json = nlohmann::json;
class ICamera {
 public:
  virtual void GetFrame(cv::Mat& mat) = 0;
  virtual ~ICamera() = default;
};
}  // namespace camera
