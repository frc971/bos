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
#define LR 0.05
#define EPOCHS 100
#define NORMALIZATION 1000
#define BETA 0.3

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
    normalized_camera_matrix_right_ = camera_matrix_right_ / NORMALIZATION;
    normalized_camera_matrix_right_(0, 0) = 1;
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
    normalized_camera_matrix_left_ = camera_matrix_left_ / NORMALIZATION;
    normalized_camera_matrix_left_(0, 0) = 1;
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

auto ProjectPoints(const frc::Pose3d& camera_pose, const frc::Pose3d& tag_pose,
                   const Eigen::Matrix3d& camera_matrix,
                   const Eigen::Matrix4d& camera_to_robot, int corner_index)
    -> Eigen::Vector3d {
  auto feild_to_robot = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag =
      camera_to_robot * feild_to_robot.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];
  return normalized_point;
}

auto CalculateLoss(const Eigen::Matrix4d& robot_to_feild,
                   const Eigen::Matrix4d& feild_to_tag,
                   const Eigen::Matrix4d& camera_to_robot,
                   const Eigen::Matrix3d& camera_matrix,
                   const Eigen::Vector3d& image_point, int corner_index)
    -> double {

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_robot * robot_to_feild * feild_to_tag *
      rotate_yaw *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  return normalized_points_d.array().square().sum();
}

auto CalculateDerivative(const Eigen::Matrix4d& robot_to_feild,
                         const Eigen::Matrix4d& feild_to_tag,
                         const Eigen::Matrix4d& camera_to_robot,
                         const Eigen::Matrix3d& camera_matrix,
                         const Eigen::Vector3d& image_point, int corner_index)
    -> Eigen::Matrix4d {
  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_robot * robot_to_feild * feild_to_tag *
      rotate_yaw *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  // clang-format off
  auto projected_point_d = (Eigen::Vector3d() << 
    (-normalized_points_d[1] * projected_point[1]) / (projected_point[0] * projected_point[0]) + 
    (-normalized_points_d[2] * projected_point[1]) / (projected_point[0] * projected_point[0]),
    normalized_points_d[1] / projected_point[0],
    normalized_points_d[2] / projected_point[0]
  ).finished();
  // clang-format on

  auto camera_matrix_xd = camera_matrix.transpose() * projected_point_d;

  auto PI_xd = PI.transpose() * camera_matrix_xd;

  auto camera_to_robot_xd = camera_to_robot.transpose() * PI_xd;

  auto robot_to_feild_d =
      camera_to_robot_xd *

      (feild_to_tag * rotate_yaw *
       localization::kapriltag_corners_eigen_homogenized[corner_index])
          .transpose();

  return robot_to_feild_d;
}

// TODO: Tolerance is quite high. Find out what causes the loss in precision
void CheckIsEqual(cv::Point2d image_point, Eigen::Vector3d projected_points,
                  double tolerance = 1) {
  EXPECT_NEAR(image_point.x, projected_points[1], tolerance);
  EXPECT_NEAR(image_point.y, projected_points[2], tolerance);
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
      auto projected_points =
          ProjectPoints(square_solver_solution.pose,
                        kapriltag_layout.GetTagPose(detection.tag_id).value(),
                        camera_matrix_right_, camera_to_robot_right_, j);

      auto projected_points_normalized = ProjectPoints(
          square_solver_solution.pose,
          kapriltag_layout.GetTagPose(detection.tag_id).value(),
          normalized_camera_matrix_right_, camera_to_robot_right_, j);

      CheckIsEqual(detection.corners[j], projected_points, 1);
      CheckIsEqual(detection.corners[j] / NORMALIZATION,
                   projected_points_normalized, 1.0 / NORMALIZATION);
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
      auto image_point = (Eigen::Vector3d() << 1,
                          detection.corners[corner_index].x / NORMALIZATION,
                          detection.corners[corner_index].y / NORMALIZATION)
                             .finished();

      robot_to_feild_d += CalculateDerivative(
          robot_to_feild, feild_to_tag, camera_to_robot_right_,
          normalized_camera_matrix_right_, image_point, corner_index);

      loss += CalculateLoss(
          robot_to_feild, feild_to_tag, camera_to_robot_right_,
          normalized_camera_matrix_right_, image_point, corner_index);
    }
    robot_to_feild -= robot_to_feild_d * LR;
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
  cv::Mat image = cv::imread("/bos-logs/log181/right/20.411762.jpg");
  // LOG(INFO) << detections.size();
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_right_->GetTagDetections(timestamped_frame);

  // PCHECK(detections.size() > 1)
  //     << "Need at least two detections to test differnt tag positions";

  auto square_solver_solution =
      square_solver_right_->EstimatePosition(detections)[0];

  auto feild_to_robot = square_solver_solution.pose.ToMatrix();

  JointSolver::DifferntiableTransform3d robot_to_feild(
      feild_to_robot.inverse());

  // Noise
  robot_to_feild.t_x += 0.1;
  robot_to_feild.t_y += 0.1;
  robot_to_feild.t_z += 0.1;

  robot_to_feild.r_x += 0.1;
  robot_to_feild.r_y += 0.1;
  robot_to_feild.r_z += 0.1;

  tape_type tape;

  double loss = 0;
  utils::Timer solve_timer("solve", false);
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
        auto image_point = (Eigen::Vector3d() << 1,
                            detection.corners[corner_index].x / NORMALIZATION,
                            detection.corners[corner_index].y / NORMALIZATION)
                               .finished();

        auto robot_to_feild_d = CalculateDerivative(
            robot_to_feild_eigen, feild_to_tag, camera_to_robot_right_,
            normalized_camera_matrix_right_, image_point, corner_index);

        robot_to_feild.RegisterOutputs(tape);
        derrivative =
            derrivative + robot_to_feild.BackPropagate(robot_to_feild_d, tape);

        loss += CalculateLoss(
            robot_to_feild_eigen, feild_to_tag, camera_to_robot_right_,
            normalized_camera_matrix_right_, image_point, corner_index);

        tape.newRecording();
      }
    }
    robot_to_feild.Apply(derrivative, LR);
  }
  ASSERT_LT(solve_timer.Stop(), 0.1);
  ASSERT_LT(loss, 0.01);
}
TEST_F(ForwardTest, TestMultiCameraBackpropagation) {  // NOLINT
  cv::Mat image_right = cv::imread("/bos-logs/log181/right/20.411762.jpg");

  cv::Mat image_left = cv::imread("/bos-logs/log181/left/20.935361.jpg");
  std::map<std::string, std::vector<tag_detection_t>> detections;
  {
    timestamped_frame_t timestamped_frame{
        .frame = std::move(image_right), .timestamp = 0, .invalid = false};
    detections[camera_constant_right_.name] =
        detector_right_->GetTagDetections(timestamped_frame);
    CHECK(!detections[camera_constant_right_.name].empty());
  }
  {
    timestamped_frame_t timestamped_frame{
        .frame = std::move(image_left), .timestamp = 0, .invalid = false};
    detections[camera_constant_left_.name] =
        detector_left_->GetTagDetections(timestamped_frame);
    CHECK(!detections[camera_constant_left_.name].empty());
  }

  auto square_solver_solution = square_solver_right_->EstimatePosition(
      detections[camera_constant_right_.name])[0];

  auto feild_to_robot = square_solver_solution.pose.ToMatrix();
  JointSolver::DifferntiableTransform3d robot_to_feild(
      feild_to_robot.inverse());

  // Noise
  robot_to_feild.t_x += 0.1;
  robot_to_feild.t_y += 0.1;
  robot_to_feild.t_z += 0.1;

  robot_to_feild.r_x += 0.1;
  robot_to_feild.r_y += 0.1;
  robot_to_feild.r_z += 0.1;

  tape_type tape;

  double loss = 0;
  std::vector<camera_constant_t> camera_constants{camera_constant_left_,
                                                  camera_constant_right_};
  utils::Timer solve_timer("solve", false);
  JointSolver::transform3d_derrivative_t velocity;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    loss = 0;
    JointSolver::transform3d_derrivative_t derrivative;
    for (const auto& camera_constant : camera_constants) {

      auto camera_to_robot =
          utils::ExtrinsicsJsonToCameraToRobot(
              utils::ReadExtrinsics(camera_constant.extrinsics_path.value()))
              .ToMatrix();

      auto camera_matrix = CameraMatrixFromJson<Eigen::Matrix3d>(
          ReadIntrinsics(camera_constant.intrinsics_path.value()));
      Eigen::Matrix3d normalized_camera_matrix = camera_matrix / NORMALIZATION;
      normalized_camera_matrix(0, 0) = 1;

      for (const auto& detection : detections[camera_constant.name]) {
        auto feild_to_tag =
            kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

        for (int corner_index = 0; corner_index < 4; corner_index++) {
          robot_to_feild.CalculateMatrix();
          robot_to_feild.RegisterInputs(tape);
          const auto robot_to_feild_eigen = robot_to_feild.ToEigen();
          auto image_point = (Eigen::Vector3d() << 1,
                              detection.corners[corner_index].x / NORMALIZATION,
                              detection.corners[corner_index].y / NORMALIZATION)
                                 .finished();

          auto robot_to_feild_d = CalculateDerivative(
              robot_to_feild_eigen, feild_to_tag, camera_to_robot,
              normalized_camera_matrix, image_point, corner_index);

          robot_to_feild.RegisterOutputs(tape);
          derrivative = derrivative +
                        robot_to_feild.BackPropagate(robot_to_feild_d, tape);

          loss += CalculateLoss(robot_to_feild_eigen, feild_to_tag,
                                camera_to_robot, normalized_camera_matrix,
                                image_point, corner_index);

          tape.newRecording();
        }
      }
    }
    velocity = (velocity * BETA) + derrivative;
    robot_to_feild.Apply(velocity, LR);
  }
  ASSERT_LT(solve_timer.Stop(), 0.1);  // The test is very unoptimized
  ASSERT_LT(loss, 0.01);
}
