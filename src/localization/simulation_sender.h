#pragma once

#include "src/camera/cscore_streamer.h"
#include "src/localization/position_sender.h"
#include "src/utils/pch.h"

namespace localization {
// Sends position estimates from the orin to the rio. It is safe to call Send() on multiple threads because of the mutex
class SimulationSender : public IPositionSender {
 public:
  SimulationSender(const std::string& name, int port);
  void Send(const std::vector<localization::position_estimate_t>& detections)
      override;
  ~SimulationSender() override = default;

 private:
  cv::Mat field_image_;
  camera::CscoreStreamer streamer_;
};
}  // namespace localization
