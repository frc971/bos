#pragma once
#include <vpi/Types.h>
#include <vpi/algo/AprilTags.h>
#include "src/localization/apriltag_detector.h"
#include "src/utils/pch.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {

// NEEDS TO BE TESTED
class NvidiaAprilTagDetector : IAprilTagDetector {
 public:
  NvidiaAprilTagDetector(int image_width, int image_height,
                         nlohmann::json intrinsics,
                         VPIAprilTagDecodeParams params, VPIBackend backend,
                         int max_detections = 16, bool verbose = false);
  ~NvidiaAprilTagDetector() override;
  auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<tag_detection_t> override;

 private:
  VPIAprilTagDecodeParams params_;
  VPIBackend backend_;
  int max_detections_;
  VPIImage input_;
  VPICameraIntrinsic intrinsics_;
  VPIPayload payload_;
  VPIArray detections_;
  VPIArray poses_;
  VPIStream stream_;
};
}  // namespace localization
