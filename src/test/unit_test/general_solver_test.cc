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
  static constexpr camera::Camera config = camera::DEV_ORIN;
  localization::SquareSolver square_solver;
  localization::MultiTagSolver multitag_solver;
  localization::OpenCVAprilTagDetector detector;
  GeneralSolverTest()
      : square_solver(config),
        multitag_solver(config),
        detector(
            utils::GetJson(camera::camera_constants[config].intrinsics_path)) {}
};

TEST_F(GeneralSolverTest, Basic) {
  const localization::position_estimate_t square_solve_estimate =
      square_solver.EstimatePosition(test_utils::fake_detections)[0];
  const localization::position_estimate_t multitag_solve_estimate =
      multitag_solver.EstimatePosition(test_utils::fake_detections)[0];
  EXPECT_EQ(square_solve_estimate, multitag_solve_estimate);
}

TEST_F(GeneralSolverTest, RealImage) {
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
