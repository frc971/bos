#pragma once

#include "src/localization/position.h"
#include "src/utils/pch.h"

namespace localization {
class IPositionSender {
 public:
  void Send(const std::vector<localization::position_estimate_t>& detections) {
    for (const auto& detection : detections) {
      Send(detection);
    }
  }
  virtual auto Send(const localization::position_estimate_t& detection)
      -> void = 0;
  virtual ~IPositionSender() = default;
};
}  // namespace localization
