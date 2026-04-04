#include "src/localization/simulation_sender.h"
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

static const std::string kfield_image_path =
    "/bos/constants/misc/2026field.png";

void DrawArrow(cv::Mat& img, cv::Point start, double angle,
               double length = 50.0,
               const cv::Scalar& color = cv::Scalar(0, 0, 255),
               int thickness = 5) {
  cv::Point end;
  end.x = static_cast<int>(start.x + length * std::cos(angle));
  end.y = static_cast<int>(start.y + length * std::sin(angle));

  cv::arrowedLine(img, start, end, color, thickness, 8, 0, 0.5);
}

SimulationSender::SimulationSender(const std::string& name, int port)
    : field_image_(cv::imread(kfield_image_path)),
      streamer_(name, port, 30, field_image_) {
  int width = field_image_.cols;
  int height = field_image_.rows;
  int crop = 270;
  cv::Rect roi(crop, 0, width - 2 * crop, height);
  field_image_ = field_image_(roi);
}

void SimulationSender::Send(
    const std::vector<localization::position_estimate_t>& detections) {
  if (detections.empty()) {
    return;
  }

  cv::Mat canvas = field_image_.clone();
  for (const auto& detection : detections) {
    LOG(INFO) << detection.pose.X().value() << detection.pose.Y().value();
    DrawArrow(
        canvas,
        cv::Point(field_image_.cols * (detection.pose.X().value() / 16.46),
                  field_image_.rows * (detection.pose.Y().value() / 8.23)),
        detection.pose.ToPose2d().Rotation().Radians().value());
  }
  streamer_.WriteFrame(canvas);
}
}  // namespace localization
