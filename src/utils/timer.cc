#include "timer.h"
#include <chrono>
#include <iostream>
namespace utils {
Timer::Timer(std::string name, bool print)
    : name_(name),
      print_(print),
      stopped_(false),
      start_(std::chrono::high_resolution_clock::now()) {}

double Timer::Stop() {
  auto end = std::chrono::high_resolution_clock::now();
  double time =
      std::chrono::duration_cast<std::chrono::duration<double>>(end - start_)
          .count();
  if (print_) {
    std::cout << name_ << " took  " << time << std::endl;
  }
  stopped_ = true;
  return time;
}
Timer::~Timer() {
  if (stopped_) {
    return;
  }
  auto end = std::chrono::high_resolution_clock::now();
  double time =
      std::chrono::duration_cast<std::chrono::duration<double>>(end - start_)
          .count();
  if (print_) {
    std::cout << name_ << " took  " << time << "s" << std::endl;
  }
}

}  // namespace utils
