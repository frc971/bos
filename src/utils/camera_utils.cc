#include "src/utils/camera_utils.h"
#include <iostream>
#include "src/utils/log.h"
namespace utils {
auto read_intrinsics(const std::string& path) -> nlohmann::json {
  nlohmann::json intrinsics;

  std::ifstream intrinsics_file(path);
  if (!intrinsics_file.is_open()) {
    LOG(FATAL) << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }
  return intrinsics;
}

auto read_extrinsics(const std::string& path) -> nlohmann::json {
  nlohmann::json extrinsics;
  std::ifstream extrinsics_file(path);
  if (!extrinsics_file.is_open()) {
    LOG(FATAL) << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    extrinsics_file >> extrinsics;
  }
  return extrinsics;
}
}  // namespace utils
