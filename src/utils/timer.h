#pragma once
#include <chrono>
#include <string>
namespace utils {
class Timer {
 public:
  Timer(std::string name, bool print = true);
  ~Timer();
  double Stop();

 private:
  std::string name_;
  bool print_;
  bool stopped_;
  std::chrono::system_clock::time_point start_;
};

}  // namespace utils
