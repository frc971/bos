#pragma once
#include <chrono>
#include <string>
namespace utils {
// Utility class used to get the amount of time it takes for a block of code to be run.
// Timer starts when the object is construcuted and stops when Stop() is called or when
// the class gets destructed (useally when the object gets out of scope)
class Timer {
 public:
  Timer(std::string name, bool print = true);
  ~Timer();
  auto Stop() -> double;

 private:
  std::string name_;
  bool print_;
  bool stopped_;
  std::chrono::system_clock::time_point start_;
};

}  // namespace utils
