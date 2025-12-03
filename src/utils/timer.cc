#include "timer.h"
#include <chrono>
#include <iostream>
namespace utils {
Timer::Timer(std::string name)
    : name_(name),
      stopped_(false),
      start_(std::chrono::high_resolution_clock::now()) {}

void Timer::Stop() {
  auto end = std::chrono::high_resolution_clock::now();
  double time =
      std::chrono::duration_cast<std::chrono::duration<double>>(end - start_)
          .count();
  std::cout << name_ << " took  " << time << std::endl;
  stopped_ = true;
}
Timer::~Timer() {
  if (stopped_) {
    return;
  }
  auto end = std::chrono::high_resolution_clock::now();
  double time =
      std::chrono::duration_cast<std::chrono::duration<double>>(end - start_)
          .count();
  std::cout << name_ << " took  " << time << "s" << std::endl;
}

}  // namespace utils
