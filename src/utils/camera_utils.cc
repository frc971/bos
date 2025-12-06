#include "src/utils/camera_utils.h"
#include <fstream>
#include <iostream>
namespace utils {
nlohmann::json read_intrinsics(std::string path) {
  nlohmann::json intrinsics;

  std::ifstream intrinsics_file(path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }
  return intrinsics;
}

nlohmann::json read_extrinsics(std::string path) {
  nlohmann::json extrinsics;
  std::ifstream extrinsics_file(path);
  if (!extrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open extrinsics file: " << path << std::endl;
  } else {
    extrinsics_file >> extrinsics;
  }
  return extrinsics;
}
}  // namespace utils
