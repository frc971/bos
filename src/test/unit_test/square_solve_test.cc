#include <gtest/gtest.h>
#include <algorithm>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/test/unit_test/unit_test_utils.h"
#include "src/utils/camera_utils.h"
#include "src/utils/transform.h"

constexpr double ERROR_MARGIN = 0.001;

class SquareSolverTest : public ::testing::Test {
 protected:
  // static constexpr camera::Camera config = camera::DEV_ORIN;
  camera::camera_constant_t camera_constant;
  localization::SquareSolver square_solver;
  localization::OpenCVAprilTagDetector detector;
  SquareSolverTest()
      : camera_constant(
            camera::GetCameraConstants("/bos/constants/camera_constants.json")
                .at("dev_orin")),
        square_solver(camera_constant),
        detector(
            utils::ReadIntrinsics(camera_constant.intrinsics_path.value())) {}
};

TEST_F(SquareSolverTest, Basic) {  // NOLINT
  const localization::position_estimate_t old_estimate =
      square_solver.EstimatePosition(test_utils::fake_detections)[0];
  EXPECT_NEAR(old_estimate.pose.Rotation().Angle().value(), M_PI, ERROR_MARGIN);
}

TEST_F(SquareSolverTest, Math) {  // NOLINT
  const cv::Mat image = cv::imread("/bos/constants/misc/apriltag.jpg");
  camera::timestamped_frame_t frame{.frame = image, .timestamp = 0.0};
  const std::vector<localization::tag_detection_t> detections =
      detector.GetTagDetections(frame);
  const localization::position_estimate_t new_estimate =
      square_solver.EstimatePositionNew(test_utils::fake_detections)[0];
  const localization::position_estimate_t old_estimate =
      square_solver.EstimatePosition(test_utils::fake_detections)[0];
  EXPECT_EQ(new_estimate, old_estimate);
}
