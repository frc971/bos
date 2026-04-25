#include <gtest/gtest.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include <filesystem>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

#define IMAGE_STRIDE 4
#define LOG_PATH "/bos-logs/log181/right"
#define LR 0.01
#define EPOCHS 1000
#define NORMALIZATION 100

namespace fs = std::filesystem;

using camera::camera_constant_t;
using camera::GetCameraConstants;
using camera::timestamped_frame_t;
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

using transform3d_derrivative_t = struct Transfrom3dDerrivative {
  double t_x = 0;
  double t_y = 0;
  double t_z = 0;
  double r_x = 0;
  double r_y = 0;
  double r_z = 0;

  auto operator+(const Transfrom3dDerrivative other) -> Transfrom3dDerrivative {
    return Transfrom3dDerrivative{.t_x = t_x + other.t_x,
                                  .t_y = t_y + other.t_y,
                                  .t_z = t_z + other.t_z,
                                  .r_x = r_x + other.r_x,
                                  .r_y = r_y + other.r_y,
                                  .r_z = r_z + other.r_z};
  }
};

auto operator<<(std::ostream& os, const Transfrom3dDerrivative& v)
    -> std::ostream& {
  os << "tx: " << v.t_x << "\n";
  os << "ty: " << v.t_y << "\n";
  os << "tz: " << v.t_z << "\n";

  os << "rx: " << v.r_x << "\n";
  os << "ry: " << v.r_y << "\n";
  os << "rz: " << v.r_z << "\n";
  return os;
}

struct DifferntiableTransform3d {

  // Translation in meters, rotation in radians
  AD t_x;
  AD t_y;
  AD t_z;
  AD r_x;
  AD r_y;
  AD r_z;
  std::array<std::array<AD, 4>, 4> matrix;

  DifferntiableTransform3d(frc::Pose3d pose)
      : t_x(pose.Translation().X().value()),
        t_y(pose.Translation().Y().value()),
        t_z(pose.Translation().Z().value()),
        r_x(pose.Rotation().X().value()),
        r_y(pose.Rotation().Y().value()),
        r_z(pose.Rotation().Z().value()) {}

  DifferntiableTransform3d(frc::Transform3d pose)
      : t_x(pose.Translation().X().value()),
        t_y(pose.Translation().Y().value()),
        t_z(pose.Translation().Z().value()),
        r_x(pose.Rotation().X().value()),
        r_y(pose.Rotation().Y().value()),
        r_z(pose.Rotation().Z().value()) {}

  DifferntiableTransform3d(Eigen::Matrix4d matrix)
      : t_x(matrix(0, 3)), t_y(matrix(1, 3)), t_z(matrix(2, 3)) {
    Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d euler = R.canonicalEulerAngles(2, 1, 0);
    r_x = euler(2);
    r_y = euler(1);
    r_z = euler(0);
  }

  void Update(transform3d_derrivative_t derrivative) {
    t_x -= derrivative.t_x * LR;
    t_y -= derrivative.t_y * LR;
    t_z -= derrivative.t_z * LR;

    r_x -= derrivative.r_x * LR;
    r_y -= derrivative.r_y * LR;
    r_z -= derrivative.r_z * LR;
  }

  void RegisterInputs(tape_type& tape) {
    tape.registerInput(r_x);
    tape.registerInput(r_y);
    tape.registerInput(r_z);

    tape.registerInput(t_x);
    tape.registerInput(t_y);
    tape.registerInput(t_z);
  }

  void RegisterOutputs(tape_type& tape) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        tape.registerOutput(matrix[i][j]);
      }
    }
  }

  auto ToEigen() -> Eigen::Matrix4d const {
    // clang-format off
    return (Eigen::Matrix<double, 4, 4>() << 
      matrix[0][0].value(), matrix[0][1].value(), matrix[0][2].value(), matrix[0][3].value(),
      matrix[1][0].value(), matrix[1][1].value(), matrix[1][2].value(), matrix[1][3].value(),
      matrix[2][0].value(), matrix[2][1].value(), matrix[2][2].value(), matrix[2][3].value(),
      matrix[3][0].value(), matrix[3][1].value(), matrix[3][2].value(), matrix[3][3].value())
      .finished();
    // clang-format on
  }
  void CalculateMatrix() {
    AD cos_x = xad::cos(r_x);
    AD cos_y = xad::cos(r_y);
    AD cos_z = xad::cos(r_z);

    AD sin_x = xad::sin(r_x);
    AD sin_y = xad::sin(r_y);
    AD sin_z = xad::sin(r_z);

    // Rotation
    matrix[0][0] = cos_z * cos_y;
    matrix[0][1] = cos_z * sin_y * sin_x - sin_z * cos_x;
    matrix[0][2] = cos_z * sin_y * cos_x + sin_z * sin_x;

    matrix[1][0] = sin_z * cos_y;
    matrix[1][1] = sin_z * sin_y * sin_x + cos_z * cos_x;
    matrix[1][2] = sin_z * sin_y * cos_x - cos_z * sin_x;

    matrix[2][0] = -sin_y;
    matrix[2][1] = cos_y * sin_x;
    matrix[2][2] = cos_y * cos_x;

    // Translation
    matrix[0][3] = t_x;
    matrix[1][3] = t_y;
    matrix[2][3] = t_z;

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }

  auto BackPropagate(const Eigen::Matrix4d& next_derrivative, tape_type& tape)
      -> transform3d_derrivative_t {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        tape.registerOutput(matrix[i][j]);
        derivative(matrix[i][j]) = next_derrivative(i, j);
      }
    }
    tape.computeAdjoints();
    transform3d_derrivative_t derrivative{
        .t_x = xad::derivative(t_x),
        .t_y = xad::derivative(t_y),
        .t_z = xad::derivative(t_z),
        .r_x = xad::derivative(r_x),
        .r_y = xad::derivative(r_y),
        .r_z = xad::derivative(r_z),
    };
    return derrivative;
  }
  void Apply(const transform3d_derrivative_t& derrivative) {
    t_x -= derrivative.t_x * LR;
    t_y -= derrivative.t_y * LR;
    t_z -= derrivative.t_z * LR;

    r_x -= derrivative.r_x * LR;
    r_y -= derrivative.r_y * LR;
    r_z -= derrivative.r_z * LR;
  }
};

auto operator<<(std::ostream& os, const DifferntiableTransform3d& v)
    -> std::ostream& {
  os << "tx: " << v.t_x << "\n";
  os << "ty: " << v.t_y << "\n";
  os << "tz: " << v.t_z << "\n";

  os << "rx: " << v.r_x << "\n";
  os << "ry: " << v.r_y << "\n";
  os << "rz: " << v.r_z << "\n";
  return os;
}

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
    camera_constant_ = GetCameraConstants().at("second_bot_right");
    camera_constant_.extrinsics_path =
        "/bos/constants/misc/dev_orin_extrinsics.json";

    square_solver_ = std::make_unique<SquareSolver>(camera_constant_);
    detector_ = std::make_unique<OpenCVAprilTagDetector>(
        camera_constant_.frame_height.value(),
        camera_constant_.frame_height.value(),
        ReadIntrinsics(camera_constant_.intrinsics_path.value()));
    camera_matrix_ = CameraMatrixFromJson<Eigen::Matrix3d>(
        ReadIntrinsics(camera_constant_.intrinsics_path.value()));
    normalized_camera_matrix_ = camera_matrix_ / NORMALIZATION;
    normalized_camera_matrix_(0, 0) = 1;
  }

  void TearDown() override {}

  camera_constant_t camera_constant_;
  std::unique_ptr<SquareSolver> square_solver_;
  std::unique_ptr<OpenCVAprilTagDetector> detector_;
  Eigen::Matrix3d camera_matrix_;
  Eigen::Matrix3d normalized_camera_matrix_;
};

auto ProjectPoints(const frc::Pose3d& camera_pose, const frc::Pose3d& tag_pose,
                   const Eigen::Matrix3d& camera_matrix, int corner_index)
    -> Eigen::Vector3d {
  auto feild_to_camera = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];
  return normalized_point;
}

auto CalculateLoss(const Eigen::Matrix4d& camera_to_tag,
                   const Eigen::Matrix3d& camera_matrix,
                   const Eigen::Vector3d& image_point, int corner_index)
    -> double {
  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  return normalized_points_d.array().square().sum();
}

auto CalculateDerivative(const Eigen::Matrix4d& camera_to_tag,
                         const Eigen::Matrix3d& camera_matrix,
                         const Eigen::Vector3d& image_point, int corner_index)
    -> Eigen::Matrix4d {

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_tag *
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

  auto camera_to_tag_d =
      PI_xd * localization::kapriltag_corners_eigen_homogenized[corner_index]
                  .transpose();
  return camera_to_tag_d;
}

// TODO: Tolerance is quite high. Find out what causes the loss in precision
void CheckIsEqual(cv::Point2d image_point, Eigen::Vector3d projected_points,
                  double tolerance = 1) {
  EXPECT_NEAR(image_point.x, projected_points[1], tolerance);
  EXPECT_NEAR(image_point.y, projected_points[2], tolerance);
}

TEST_F(ForwardTest, ProjectTest) {  // NOLINT
  // [u, v] = camera_matrix * PI * camera_to_tag * tag_corners
  // feild_to_camera = feild_to_tag * 180_yaw * camera_to_tag.inv
  // feild_to_camera * camera_to_tag = feild_to_tag * 180_yaw
  // camera_to_tag = feild_to_camera.inv * feild_to_tag * 180_yaw

  std::vector<fs::path> image_paths;
  for (const auto& file : fs::directory_iterator(LOG_PATH)) {
    image_paths.push_back(file.path());
  }
  std::sort(image_paths.begin(), image_paths.end());

  for (size_t i = 0; i < image_paths.size(); i += IMAGE_STRIDE) {
    cv::Mat image = cv::imread(image_paths[i]);
    timestamped_frame_t timestamped_frame{
        .frame = std::move(image), .timestamp = 0, .invalid = false};
    auto detections = detector_->GetTagDetections(timestamped_frame);
    if (detections.empty()) {
      continue;
    }
    auto detection = detections[0];

    auto square_solver_solution =
        square_solver_->EstimatePosition({detection})[0];

    for (int j = 0; j < 4; j++) {
      auto projected_points =
          ProjectPoints(square_solver_solution.pose,
                        kapriltag_layout.GetTagPose(detection.tag_id).value(),
                        camera_matrix_, j);

      auto projected_points_normalized =
          ProjectPoints(square_solver_solution.pose,
                        kapriltag_layout.GetTagPose(detection.tag_id).value(),
                        normalized_camera_matrix_, j);

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
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  auto feild_to_tag =
      kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

  auto feild_to_camera = square_solver_solution.pose.ToMatrix();

  Eigen::Matrix4d camera_to_tag =
      feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  // Noise
  camera_to_tag(0, 0) += 0.1;
  camera_to_tag(0, 3) += 0.1;
  camera_to_tag(0, 2) += 0.1;
  camera_to_tag(1, 3) += 0.1;

  double loss = 0;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    Eigen::Matrix4d camera_to_tag_d = Eigen::Matrix4d::Zero();
    loss = 0;
    for (int corner_index = 0; corner_index < 4; corner_index++) {
      auto image_point = (Eigen::Vector3d() << 1,
                          detection.corners[corner_index].x / NORMALIZATION,
                          detection.corners[corner_index].y / NORMALIZATION)
                             .finished();

      camera_to_tag_d += CalculateDerivative(
          camera_to_tag, normalized_camera_matrix_, image_point, corner_index);

      loss += CalculateLoss(camera_to_tag, normalized_camera_matrix_,
                            image_point, corner_index);
    }
    camera_to_tag -= camera_to_tag_d * LR;
  }
  ASSERT_LT(loss, 0.001);
}

TEST_F(ForwardTest, TestTransfrom3dConstructor) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  DifferntiableTransform3d transform_from_pose(square_solver_solution.pose);
  DifferntiableTransform3d transform_from_eigen(
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
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  DifferntiableTransform3d transform(square_solver_solution.pose);
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

TEST_F(ForwardTest, TestBackpropagation) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  auto feild_to_tag =
      kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

  auto feild_to_camera = square_solver_solution.pose.ToMatrix();

  DifferntiableTransform3d camera_to_tag(feild_to_camera.inverse() *
                                         feild_to_tag * rotate_yaw);

  // Noise
  camera_to_tag.t_x += 0.1;
  camera_to_tag.t_y += 0.1;
  camera_to_tag.t_z += 0.1;

  camera_to_tag.r_x += 0.1;
  camera_to_tag.r_y += 0.1;
  camera_to_tag.r_z += 0.1;

  tape_type tape;

  double loss = 0;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    loss = 0;
    transform3d_derrivative_t derrivative;

    for (int corner_index = 0; corner_index < 4; corner_index++) {
      camera_to_tag.CalculateMatrix();
      camera_to_tag.RegisterInputs(tape);
      auto image_point = (Eigen::Vector3d() << 1,
                          detection.corners[corner_index].x / NORMALIZATION,
                          detection.corners[corner_index].y / NORMALIZATION)
                             .finished();

      auto camera_to_tag_d = CalculateDerivative(camera_to_tag.ToEigen(),
                                                 normalized_camera_matrix_,
                                                 image_point, corner_index);
      camera_to_tag.RegisterOutputs(tape);
      derrivative =
          derrivative + camera_to_tag.BackPropagate(camera_to_tag_d, tape);

      loss += CalculateLoss(camera_to_tag.ToEigen(), normalized_camera_matrix_,
                            image_point, corner_index);
      tape.newRecording();
    }
    camera_to_tag.Apply(derrivative);
  }
  ASSERT_LT(loss, 0.001);
}
