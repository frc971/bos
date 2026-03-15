#include <gtest/gtest.h>
#include <algorithm>
#include "src/localization/joint_solver.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/test/unit_test/unit_test_utils.h"
#include "src/utils/camera_utils.h"
#include "src/utils/transform.h"

class GeneralSolverTest : public ::testing::Test {
 protected:
  camera::camera_constant_t camera_constant;
  localization::MultiTagSolver multitag_solver;
  localization::SquareSolver square_solver;
  localization::OpenCVAprilTagDetector detector;
  GeneralSolverTest()
      : camera_constant(camera::GetCameraConstants().at("dev_orin")),
        multitag_solver(camera_constant),
        square_solver(camera_constant),
        detector(
            utils::ReadIntrinsics(camera_constant.intrinsics_path.value())) {}
};

TEST_F(GeneralSolverTest, Basic) {  // NOLINT
  const localization::position_estimate_t square_solve_estimate =
      square_solver.EstimatePosition(test_utils::fake_detections)[0];
  const localization::position_estimate_t multitag_solve_estimate =
      multitag_solver.EstimatePosition(test_utils::fake_detections)[0];
  EXPECT_EQ(square_solve_estimate, multitag_solve_estimate);
}

TEST_F(GeneralSolverTest, RealImage) {  // NOLINT
  const cv::Mat image = cv::imread("/bos/constants/misc/apriltag.jpg");
  camera::timestamped_frame_t frame{.frame = image, .timestamp = 0.0};
  const std::vector<localization::tag_detection_t> detections =
      detector.GetTagDetections(frame);
  const localization::position_estimate_t square_solve_estimate =
      square_solver.EstimatePosition(detections)[0];
  const localization::position_estimate_t multitag_solve_estimate =
      multitag_solver.EstimatePosition(detections)[0];
  EXPECT_EQ(square_solve_estimate, multitag_solve_estimate);
}
