#pragma once

#include "src/localization/position.h"
#include "src/utils/pch.h"

namespace localization {
class IPositionSender {
 public:
  virtual auto Send(
      const std::vector<localization::position_estimate_t>& detections,
      double latency) -> void = 0;
  virtual ~IPositionSender() = default;
};
}  // namespace localization
