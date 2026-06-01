#pragma once

#include "src/camera/cscore_streamer.h"
#include "src/localization/position_sender.h"
#include "src/utils/pch.h"

namespace localization {
class SimulationSender : public IPositionSender {
 public:
  SimulationSender(const std::string& name, int port);
  void Send(const localization::position_estimate_t& detection) override;
  ~SimulationSender() override = default;

 private:
  cv::Mat field_image_;
  camera::CscoreStreamer streamer_;
};
}  // namespace localization
