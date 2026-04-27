#include <gtest/gtest.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include <filesystem>
#include "src/localization/joint_solver.h"
#include "src/localization/joint_solver.h"  // NO LINT
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/timer.h"
#include "src/utils/transform.h"

#define IMAGE_STRIDE 4
#define LOG_PATH "/bos-logs/log181/right"
#define LR 0.1
#define EPOCHS 10000
#define BETA 0.99

namespace fs = std::filesystem;

using camera::camera_constant_t;
using camera::GetCameraConstants;
using camera::timestamped_frame_t;
using localization::JointSolver;
using localization::kapriltag_layout;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
using utils::ReadIntrinsics;

using mode = xad::adj<double>;
using tape_type = mode::tape_type;
using AD = mode::active_type;

// clang-format off
const Eigen::Matrix<double, 3, 4> PI =
    (Eigen::Matrix<double, 3, 4>() << 
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0).finished();

const Eigen::Matrix<double, 4, 4> rotate_yaw =
    (Eigen::Matrix<double, 4, 4>() << 
  -1, 0, 0, 0,
  0, -1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1).finished();
// clang-format on

class ForwardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    camera_constant_right_ = GetCameraConstants().at("second_bot_right");
    square_solver_right_ =
        std::make_unique<SquareSolver>(camera_constant_right_);
    detector_right_ = std::make_unique<OpenCVAprilTagDetector>(
        camera_constant_right_.frame_width.value(),
        camera_constant_right_.frame_height.value(),
        ReadIntrinsics(camera_constant_right_.intrinsics_path.value()));
    camera_matrix_right_ = CameraMatrixFromJson<Eigen::Matrix3d>(
        ReadIntrinsics(camera_constant_right_.intrinsics_path.value()));
    normalized_camera_matrix_right_ = JointSolver::NormalizeCameraMatrix(
        camera_matrix_right_, camera_constant_right_);
    camera_to_robot_right_ =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(
                camera_constant_right_.extrinsics_path.value()))
            .ToMatrix();

    camera_constant_left_ = GetCameraConstants().at("second_bot_left");
    square_solver_left_ = std::make_unique<SquareSolver>(camera_constant_left_);
    detector_left_ = std::make_unique<OpenCVAprilTagDetector>(
        camera_constant_left_.frame_width.value(),
        camera_constant_left_.frame_height.value(),
        ReadIntrinsics(camera_constant_left_.intrinsics_path.value()));
    camera_matrix_left_ = CameraMatrixFromJson<Eigen::Matrix3d>(
        ReadIntrinsics(camera_constant_left_.intrinsics_path.value()));
    normalized_camera_matrix_left_ = JointSolver::NormalizeCameraMatrix(
        camera_matrix_left_, camera_constant_left_);
    camera_to_robot_left_ =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(
                camera_constant_left_.extrinsics_path.value()))
            .ToMatrix();
  }

  void TearDown() override {}

  std::unique_ptr<OpenCVAprilTagDetector> detector_right_;
  camera_constant_t camera_constant_right_;
  std::unique_ptr<SquareSolver> square_solver_right_;
  Eigen::Matrix3d camera_matrix_right_;
  Eigen::Matrix3d normalized_camera_matrix_right_;
  Eigen::Matrix4d camera_to_robot_right_;

  std::unique_ptr<OpenCVAprilTagDetector> detector_left_;
  camera_constant_t camera_constant_left_;
  std::unique_ptr<SquareSolver> square_solver_left_;
  Eigen::Matrix3d camera_matrix_left_;
  Eigen::Matrix3d normalized_camera_matrix_left_;
  Eigen::Matrix4d camera_to_robot_left_;
};

// TODO: Tolerance is quite high. Find out what causes the loss in precision
void CheckIsEqual(cv::Point2d image_point, Eigen::Vector3d projected_points,
                  double tolerance) {
  EXPECT_NEAR(image_point.x, projected_points[1], tolerance);
  EXPECT_NEAR(image_point.y, projected_points[2], tolerance);
}

void CheckIsEqual(Eigen::Vector3d image_point, Eigen::Vector3d projected_points,
                  double tolerance) {
  EXPECT_NEAR(image_point[1], projected_points[1], tolerance);
  EXPECT_NEAR(image_point[2], projected_points[2], tolerance);
}

TEST_F(ForwardTest, TestForward) {  // NOLINT
  // [u, v] = camera_matrix * PI * camera_to_tag * tag_corners
  //
  // feild_to_camera = feild_to_tag * 180_yaw * camera_to_tag.inv
  // feild_to_camera * camera_to_tag = feild_to_tag * 180_yaw
  // camera_to_tag = feild_to_camera.inv * feild_to_tag * 180_yaw
  //
  // feild_to_robot = feild_to_tag * 180_yaw * camera_to_tag.inv * camera_to_robot
  // feild_to_robot * camera_to_robot.inv = feild_to_tag * 180_yaw * camera_to_tag.inv
  // feild_to_robot * camera_to_robot.inv * camera_to_tag = feild_to_tag * 180_yaw
  // camera_to_robot.inv * camera_to_tag = feild_to_robot.inv * feild_to_tag * 180_yaw
  // camera_to_tag = camera_to_robot * feild_to_robot.inv * feild_to_tag * 180_yaw

  std::vector<fs::path> image_paths;
  for (const auto& file : fs::directory_iterator(LOG_PATH)) {
    image_paths.push_back(file.path());
  }
  std::sort(image_paths.begin(), image_paths.end());

  for (size_t i = 0; i < image_paths.size(); i += IMAGE_STRIDE) {
    cv::Mat image = cv::imread(image_paths[i]);
    timestamped_frame_t timestamped_frame{
        .frame = std::move(image), .timestamp = 0, .invalid = false};
    auto detections = detector_right_->GetTagDetections(timestamped_frame);
    if (detections.empty()) {
      continue;
    }
    auto detection = detections[0];

    auto square_solver_solution =
        square_solver_right_->EstimatePosition({detection})[0];

    for (int j = 0; j < 4; j++) {
      auto projected_points = JointSolver::ProjectPoints(
          square_solver_solution.pose,
          kapriltag_layout.GetTagPose(detection.tag_id).value(),
          camera_matrix_right_, camera_to_robot_right_, j);

      auto projected_points_normalized = JointSolver::ProjectPoints(
          square_solver_solution.pose,
          kapriltag_layout.GetTagPose(detection.tag_id).value(),
          normalized_camera_matrix_right_, camera_to_robot_right_, j);

      CheckIsEqual(detection.corners[j], projected_points, 1);
      CheckIsEqual(JointSolver::NormalizePoint(detection.corners[j],
                                               camera_constant_right_),
                   projected_points_normalized, 0.001);
    }
  }
}

TEST_F(ForwardTest, TestDerrivative) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_right_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_right_->EstimatePosition({detection})[0];

  auto feild_to_tag =
      kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

  Eigen::Matrix4d robot_to_feild =
      square_solver_solution.pose.ToMatrix().inverse();

  // Noise
  robot_to_feild(0, 0) += 0.1;
  robot_to_feild(0, 3) += 0.1;
  robot_to_feild(0, 2) += 0.1;
  robot_to_feild(1, 3) += 0.1;

  double loss = 0;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    Eigen::Matrix4d robot_to_feild_d = Eigen::Matrix4d::Zero();
    loss = 0;
    for (int corner_index = 0; corner_index < 4; corner_index++) {

      auto image_point = JointSolver::NormalizePoint(
          detection.corners[corner_index], camera_constant_right_);

      robot_to_feild_d += JointSolver::CalculateDerivative(
          robot_to_feild, feild_to_tag, camera_to_robot_right_,
          normalized_camera_matrix_right_, image_point, corner_index);

      loss += JointSolver::CalculateLoss(
          robot_to_feild, feild_to_tag, camera_to_robot_right_,
          normalized_camera_matrix_right_, image_point, corner_index);
    }
    robot_to_feild -= robot_to_feild_d * 0.01;
  }
  ASSERT_LT(loss, 0.001);
}

TEST_F(ForwardTest, TestTransfrom3dConstructor) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_right_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_right_->EstimatePosition({detection})[0];

  JointSolver::DifferntiableTransform3d transform_from_pose(
      square_solver_solution.pose);
  JointSolver::DifferntiableTransform3d transform_from_eigen(
      square_solver_solution.pose.ToMatrix());

  const double tolerance = 1e-7;
  ASSERT_NEAR(transform_from_pose.t_x.value(), transform_from_eigen.t_x.value(),
              tolerance);
  ASSERT_NEAR(transform_from_pose.t_y.value(), transform_from_eigen.t_y.value(),
              tolerance);
  ASSERT_NEAR(transform_from_pose.t_z.value(), transform_from_eigen.t_z.value(),
              tolerance);

  ASSERT_NEAR(transform_from_pose.r_x.value(), transform_from_eigen.r_x.value(),
              tolerance);
  ASSERT_NEAR(transform_from_pose.r_y.value(), transform_from_eigen.r_y.value(),
              tolerance);
  ASSERT_NEAR(transform_from_pose.r_z.value(), transform_from_eigen.r_z.value(),
              tolerance);
}

TEST_F(ForwardTest, TestTransfrom3d) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_right_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_right_->EstimatePosition({detection})[0];

  JointSolver::DifferntiableTransform3d transform(square_solver_solution.pose);
  transform.CalculateMatrix();

  EXPECT_TRUE(square_solver_solution.pose.ToMatrix().isApprox(
      transform.ToEigen(), 1e-9));

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_NEAR(square_solver_solution.pose.ToMatrix()(i, j),
                  transform.matrix[i][j].value(), 1e-9);
    }
  }
}

TEST_F(ForwardTest, TestMultiTagBackpropagation) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/6.767740.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_right_->GetTagDetections(timestamped_frame);

  LOG(INFO) << detections.size();
  // PCHECK(detections.size() > 1)
  //     << "Need at least two detections to test differnt tag positions";

  auto square_solver_solution =
      square_solver_right_->EstimatePosition(detections)[0];

  auto feild_to_robot = square_solver_solution.pose.ToMatrix();

  JointSolver::DifferntiableTransform3d robot_to_feild(
      feild_to_robot.inverse());

  robot_to_feild.CalculateMatrix();
  LOG(INFO) << "robot_to_feild_before\n" << robot_to_feild.ToEigen();

  // Noise
  robot_to_feild.t_x += 0.1;
  robot_to_feild.t_y += 0.1;
  robot_to_feild.t_z += 0.1;

  // robot_to_feild.r_x += 0.05;
  // robot_to_feild.r_y += 0.05;
  robot_to_feild.r_z += 0.05;

  tape_type tape;

  double loss = 0;
  utils::Timer solve_timer("solve", false);
  JointSolver::transform3d_derrivative_t velocity;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    loss = 0;
    JointSolver::transform3d_derrivative_t derrivative;
    for (const auto& detection : detections) {
      auto feild_to_tag =
          kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

      for (int corner_index = 0; corner_index < 4; corner_index++) {
        robot_to_feild.CalculateMatrix();
        robot_to_feild.RegisterInputs(tape);
        const auto robot_to_feild_eigen = robot_to_feild.ToEigen();

        auto image_point = JointSolver::NormalizePoint(
            detection.corners[corner_index], camera_constant_right_);

        auto robot_to_feild_d = JointSolver::CalculateDerivative(
            robot_to_feild_eigen, feild_to_tag, camera_to_robot_right_,
            normalized_camera_matrix_right_, image_point, corner_index);

        robot_to_feild.RegisterOutputs(tape);
        derrivative =
            derrivative + robot_to_feild.BackPropagate(robot_to_feild_d, tape);

        loss += JointSolver::CalculateLoss(
            robot_to_feild_eigen, feild_to_tag, camera_to_robot_right_,
            normalized_camera_matrix_right_, image_point, corner_index);

        tape.newRecording();
      }
    }

    velocity = (velocity * BETA) + derrivative;
    robot_to_feild.Update(velocity, LR);
    if (epoch == 0) {
      LOG(INFO) << loss;
    }
  }

  robot_to_feild.CalculateMatrix();
  LOG(INFO) << "robot_to_feild_after\n" << robot_to_feild.ToEigen();
  LOG(INFO) << loss;

  ASSERT_LT(solve_timer.Stop(), 0.1);
  ASSERT_LT(loss, 0.01);
}

// TEST_F(ForwardTest, TestMultiCameraBackpropagation) {  // NOLINT
//   cv::Mat image_right = cv::imread("/bos-logs/log181/right/20.411762.jpg");
//
//   cv::Mat image_left = cv::imread("/bos-logs/log181/left/20.935361.jpg");
//   std::map<std::string, std::vector<tag_detection_t>> detections;
//   {
//     timestamped_frame_t timestamped_frame{
//         .frame = std::move(image_right), .timestamp = 0, .invalid = false};
//     detections[camera_constant_right_.name] =
//         detector_right_->GetTagDetections(timestamped_frame);
//     CHECK(!detections[camera_constant_right_.name].empty());
//   }
//   {
//     timestamped_frame_t timestamped_frame{
//         .frame = std::move(image_left), .timestamp = 0, .invalid = false};
//     detections[camera_constant_left_.name] =
//         detector_left_->GetTagDetections(timestamped_frame);
//     CHECK(!detections[camera_constant_left_.name].empty());
//   }
//
//   auto square_solver_solution = square_solver_right_->EstimatePosition(
//       detections[camera_constant_right_.name])[0];
//
//   auto feild_to_robot = square_solver_solution.pose.ToMatrix();
//   JointSolver::DifferntiableTransform3d robot_to_feild(
//       feild_to_robot.inverse());
//
//   // Noise
//   robot_to_feild.t_x += 0.1;
//   robot_to_feild.t_y += 0.1;
//   robot_to_feild.t_z += 0.1;
//
//   robot_to_feild.r_x += 0.1;
//   robot_to_feild.r_y += 0.1;
//   robot_to_feild.r_z += 0.1;
//
//   tape_type tape;
//
//   double loss = 0;
//   std::vector<camera_constant_t> camera_constants{camera_constant_left_,
//                                                   camera_constant_right_};
//   utils::Timer solve_timer("solve", false);
//   JointSolver::transform3d_derrivative_t velocity;
//   for (int epoch = 0; epoch < EPOCHS; epoch++) {
//     loss = 0;
//     JointSolver::transform3d_derrivative_t derrivative;
//     for (const auto& camera_constant : camera_constants) {
//
//       auto camera_to_robot =
//           utils::ExtrinsicsJsonToCameraToRobot(
//               utils::ReadExtrinsics(camera_constant.extrinsics_path.value()))
//               .ToMatrix();
//
//       auto camera_matrix = CameraMatrixFromJson<Eigen::Matrix3d>(
//           ReadIntrinsics(camera_constant.intrinsics_path.value()));
//       Eigen::Matrix3d normalized_camera_matrix = camera_matrix / NORMALIZATION;
//       normalized_camera_matrix(0, 0) = 1;
//
//       for (const auto& detection : detections[camera_constant.name]) {
//         auto feild_to_tag =
//             kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();
//
//         for (int corner_index = 0; corner_index < 4; corner_index++) {
//           robot_to_feild.CalculateMatrix();
//           robot_to_feild.RegisterInputs(tape);
//           const auto robot_to_feild_eigen = robot_to_feild.ToEigen();
//           auto image_point = (Eigen::Vector3d() << 1,
//                               detection.corners[corner_index].x / NORMALIZATION,
//                               detection.corners[corner_index].y / NORMALIZATION)
//                                  .finished();
//
//           auto robot_to_feild_d = JointSolver::CalculateDerivative(
//               robot_to_feild_eigen, feild_to_tag, camera_to_robot,
//               normalized_camera_matrix, image_point, corner_index);
//
//           robot_to_feild.RegisterOutputs(tape);
//           derrivative = derrivative +
//                         robot_to_feild.BackPropagate(robot_to_feild_d, tape);
//
//           loss += JointSolver::CalculateLoss(
//               robot_to_feild_eigen, feild_to_tag, camera_to_robot,
//               normalized_camera_matrix, image_point, corner_index);
//
//           tape.newRecording();
//         }
//       }
//     }
//     velocity = (velocity * BETA) + derrivative;
//     robot_to_feild.Apply(velocity, LR);
//   }
//   ASSERT_LT(solve_timer.Stop(), 0.1);  // The test is very unoptimized
//   ASSERT_LT(loss, 0.01);
// }

// TEST_F(ForwardTest, JointSolverTest) {  // NOLINT
//   cv::Mat image_right = cv::imread("/bos-logs/log181/right/20.411762.jpg");
//
//   cv::Mat image_left = cv::imread("/bos-logs/log181/left/20.935361.jpg");
//   std::map<std::string, std::vector<tag_detection_t>> detections;
//
//   {
//     timestamped_frame_t timestamped_frame{
//         .frame = std::move(image_right), .timestamp = 0, .invalid = false};
//     detections[camera_constant_right_.name] =
//         detector_right_->GetTagDetections(timestamped_frame);
//     CHECK(!detections[camera_constant_right_.name].empty());
//   }
//   {
//     timestamped_frame_t timestamped_frame{
//         .frame = std::move(image_left), .timestamp = 0, .invalid = false};
//     detections[camera_constant_left_.name] =
//         detector_left_->GetTagDetections(timestamped_frame);
//     CHECK(!detections[camera_constant_left_.name].empty());
//   }
//
//   auto square_solver_solution = square_solver_left_->EstimatePosition(
//       detections[camera_constant_left_.name])[0];
//
//   std::vector<camera_constant_t> camera_constants{camera_constant_left_,
//                                                   camera_constant_right_};
//
//   JointSolver joint_solver(camera_constants);
//   position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(detections, square_solver_solution.pose);
//
//   // LOG(INFO) << "square_solver_solution_pose\n"
//   //           << square_solver_solution.pose.ToMatrix();
//   //
//   // LOG(INFO) << "joint_solver_solution_pose\n"
//   //           << joint_solver_solution.pose.ToMatrix();
// }
