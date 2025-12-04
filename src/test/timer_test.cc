#include "src/utils/timer.h"
#include <thread>
int main() {
  utils::Timer timer_1("waiting for 0.22 ms");
  std::this_thread::sleep_for(std::chrono::milliseconds(22));
  timer_1.Stop();

  utils::Timer timer_2("destruction");
}
