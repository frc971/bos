#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "src/localization/apriltag_detector.h"
#include "src/utils/pch.h"
#include "third_party/971apriltag/apriltag.h"

namespace localization {

// Austin's GPU apriltag detection wrapper
// If given and image, with too low or too high exposure, the program will crash with 'invalid device ordinal'.
// Hopefully we can fix change the code to catch that error
class GPUAprilTagDetector : public IAprilTagDetector {
 public:
  GPUAprilTagDetector(uint image_width, uint image_height,
                      const nlohmann::json& intrinsics, bool verbose = false);
  ~GPUAprilTagDetector() override;
  auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<tag_detection_t> override;

 private:
  frc::apriltag::CameraMatrix camera_matrix_;
  frc::apriltag::DistCoeffs distortion_coefficients_;
  apriltag_detector_t* apriltag_detector_;
  std::unique_ptr<frc::apriltag::GpuDetector> gpu_detector_;
};
}  // namespace localization
