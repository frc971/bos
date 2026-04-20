#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

std::vector<localization::tag_detection_t> detections(
    {{.tag_id = 31,
      .corners = {cv::Point2d(100.0, 200.0), cv::Point2d(200.0, 200.0),
                  cv::Point2d(200.0, 100.0), cv::Point2d(100.0, 100.0)},
      .timestamp = 0.0,
      .confidence = 0.0}});

TEST(JointSolveTest, EstimatePosition) {  // NOLINT
  camera::camera_constant_t camera_constant =
      camera::GetCameraConstants().at("second_bot_left");
  std::string image_path = "/bos-logs/log181/left/9.627390.jpg";

  localization::JointSolver joint_solver({camera_constant});
  localization::SquareSolver square_solver(camera_constant);
  auto detector = std::make_unique<localization::OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));

  cv::Mat image = cv::imread(image_path);
  camera::timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};

  auto detections = detector->GetTagDetections(timestamped_frame);

  auto square_solver_solution = square_solver.EstimatePosition({detections})[0];

  utils::PrintTransformationMatrix(
      utils::EigenToCvMat(square_solver_solution.pose.ToMatrix()));
  std::cout << square_solver_solution.pose.ToMatrix() << std::endl;

  frc::Transform3d noise(
      frc::Translation3d(units::meter_t{0.067}, units::meter_t{0.1},
                         units::meter_t{-0.1}),
      {});

  square_solver_solution.pose = square_solver_solution.pose.TransformBy(noise);

  std::map<std::string, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert({camera_constant.name, detections});
  auto joint_solver_solution = joint_solver.EstimatePosition(
      associated_detections, square_solver_solution.pose);
  std::cout << "Joint solver solution: " << joint_solver_solution << std::endl;
  std::cout << "Square solver solution: " << square_solver_solution
            << std::endl;
}
