#pragma once

#include <cstdint>
namespace pathing {
class Controller {
 public:
  Controller();
  void Send();

 private:
  int64_t kDtUs = 20'000;
  double ax;
  double ay;
};

}  // namespace pathing
