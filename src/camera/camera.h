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
  static json read_intrinsics(std::string path) {
  json intrinsics;

  std::ifstream intrinsics_file(path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }
  return intrinsics;
}

  static json read_extrinsics(std::string path) {
    json extrinsics;
    std::ifstream extrinsics_file(path);
    if (!extrinsics_file.is_open()) {
      std::cerr << "Error: Cannot open extrinsics file: " << path << std::endl;
    } else {
      extrinsics_file >> extrinsics;
    }
    return extrinsics;
  }
};
}  // namespace camera
