#include <gtest/gtest.h>
#include <algorithm>
#include "src/localization/joint_solver.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/test/unit_test/unit_test_utils.h"
#include "src/utils/camera_utils.h"
#include "src/utils/transform.h"

TEST(GeneralSolverTest, Basic) {
  const camera::Camera config = camera::DEV_ORIN;
  localization::SquareSolver square_solver(config);
  localization::JointSolver joint_solver(std::vector<camera::Camera>{config});
  localization::MultiTagSolver multitag_solver(config);
  const localization::tag_detection_t fake_detection{
      .tag_id = 31, .corners = test_utils::fake_image_points};
  const std::vector<localization::tag_detection_t> fake_detections{
      fake_detection};
  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      joint_solve_input;
  joint_solve_input.insert({config, fake_detections});

  localization::position_estimate_t square_solve_estimate =
      square_solver.EstimatePosition(fake_detections)[0];
  localization::position_estimate_t multitag_solve_estimate =
      multitag_solver.EstimatePosition(fake_detections)[0];
  localization::position_estimate_t joint_solve_estimate =
      joint_solver.EstimatePosition(joint_solve_input,
                                    square_solve_estimate.pose);
  std::cout << "sq: " << square_solve_estimate << std::endl;
  std::cout << "multi: " << multitag_solve_estimate << std::endl;
  std::cout << "joint: " << joint_solve_estimate << std::endl;
  EXPECT_EQ(square_solve_estimate, multitag_solve_estimate);
  EXPECT_EQ(multitag_solve_estimate, joint_solve_estimate);
}
