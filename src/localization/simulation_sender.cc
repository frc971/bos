#include "src/localization/simulation_sender.h"
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

static const std::string kfield_image_path =
    "/bos/constants/misc/2026field.png";
SimulationSender::SimulationSender(const std::string& name, int port)
    : field_image_(cv::imread(kfield_image_path)),
      streamer_(name, port, 30, field_image_) {}

void SimulationSender::Send(
    const std::vector<localization::position_estimate_t>& detections) {}
}  // namespace localization
