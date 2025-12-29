#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <vpi/Types.h>
#include <vpi/algo/AprilTags.h>
#include <nlohmann/json.hpp>
#include "src/localization/apriltag_detector.h"
#include "third_party/971apriltag/971apriltag.h"

namespace localization {

class NvidiaAprilTagDetector : IAprilTagDetector {
 public:
  NvidiaAprilTagDetector(
      int image_width, int image_height, nlohmann::json intrinsics,
      VPIAprilTagDecodeParams params, int max_detections,
      std::vector<cv::Point3f> apriltag_dimensions = kapriltag_dimensions,
      bool verbose = false);
  ~NvidiaAprilTagDetector();
  std::vector<tag_detection_t> GetTagDetections(
      camera::timestamped_frame_t& frame) override;

 private:
  VPIAprilTagDecodeParams params_;
  int max_detections_;
  std::vector<cv::Point3f> apriltag_dimensions_;
  VPICameraIntrinsic intrinsics_;
  VPIPayload payload_;
  VPIArray detections_;
  VPIArray poses_;
  VPIStream stream_;
  bool verbose_;
};
}  // namespace localization
