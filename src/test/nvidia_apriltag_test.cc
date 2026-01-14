#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <sys/types.h>
#include <vpi/Array.h>
#include <vpi/ArrayType.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/Types.h>
#include <vpi/algo/AprilTags.h>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/localization/nvidia_apriltag_detector.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"
#include "src/utils/timer.h"

auto main() -> int {
  camera::Camera config = camera::SelectCameraConfig();
  camera::CameraSource source("nvidia_apriltag_test",
                              camera::GetCameraStream(config));
  cv::Mat mat = source.GetFrame();

  camera::CscoreStreamer streamer(source.GetName(), 4971, 30, 1080, 1080);

  localization::NvidiaAprilTagDetector detector(
      mat.cols, mat.rows,
      utils::read_intrinsics(camera::camera_constants[config].intrinsics_path),
      VPIAprilTagDecodeParams{NULL, 0, 1, VPI_APRILTAG_36H11}, VPI_BACKEND_PVA);

  camera::timestamped_frame_t timestamped_frame;
  while (true) {
    utils::Timer timer("nvidia apriltag");
    timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);

    std::vector<localization::position_estimate_t> estimates =
        detector.GetTagDetections(timestamped_frame);

    timer.Stop();
    for (auto& estimate : estimates) {
      std::cout << estimate;
      std::cout << "----------" << std::endl;
    }
  }
}
