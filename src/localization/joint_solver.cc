#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

#define IMAGE_STRIDE 4
#define LOG_PATH "/bos-logs/log181/right"
#define LR 0.05
#define EPOCHS 100
#define BETA 0.3

using frc::AprilTagFieldLayout;
using utils::CameraMatrixFromJson;
using utils::ExtrinsicsJsonToCameraToRobot;
using utils::ReadExtrinsics;
using utils::ReadIntrinsics;

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

constexpr auto square(double x) -> double {
  return x * x;
}

auto JointSolver::Transfrom3dDerivative::operator+(
    const Transfrom3dDerivative other) -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler + other.scaler,
                               .theta = theta + other.theta,
                               .t_z = t_z + other.t_z,
                               .r_x = r_x + other.r_x,
                               .r_y = r_y + other.r_y,
                               .r_z = r_z + other.r_z};
}

auto JointSolver::Transfrom3dDerivative::operator-(
    const Transfrom3dDerivative other) -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler - other.scaler,
                               .theta = theta - other.theta,
                               .t_z = t_z - other.t_z,
                               .r_x = r_x - other.r_x,
                               .r_y = r_y - other.r_y,
                               .r_z = r_z - other.r_z};
}

auto JointSolver::Transfrom3dDerivative::operator*(const double other)
    -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler * other,
                               .theta = theta * other,
                               .t_z = t_z * other,
                               .r_x = r_x * other,
                               .r_y = r_y * other,
                               .r_z = r_z * other};
}

auto JointSolver::Transfrom3dDerivative::operator+(const double other)
    -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler + other,
                               .theta = theta + other,
                               .t_z = t_z + other,
                               .r_x = r_x + other,
                               .r_y = r_y + other,
                               .r_z = r_z + other};
}

auto JointSolver::Transfrom3dDerivative::operator*(
    const Transfrom3dDerivative other) -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler * other.scaler,
                               .theta = theta * other.theta,
                               .t_z = t_z * other.t_z,
                               .r_x = r_x * other.r_x,
                               .r_y = r_y * other.r_y,
                               .r_z = r_z * other.r_z};
}

auto JointSolver::Transfrom3dDerivative::operator/(
    const Transfrom3dDerivative other) -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = scaler / other.scaler,
                               .theta = theta / other.theta,
                               .t_z = t_z / other.t_z,
                               .r_x = r_x / other.r_x,
                               .r_y = r_y / other.r_y,
                               .r_z = r_z / other.r_z};
}

auto JointSolver::Transfrom3dDerivative::sqrt() -> Transfrom3dDerivative {
  return Transfrom3dDerivative{.scaler = std::sqrt(scaler),
                               .theta = std::sqrt(theta),
                               .t_z = std::sqrt(t_z),
                               .r_x = std::sqrt(r_x),
                               .r_y = std::sqrt(r_y),
                               .r_z = std::sqrt(r_z)};
}

JointSolver::DifferentiableTransform3d::DifferentiableTransform3d(
    const Eigen::VectorXd& params)
    : scaler(params(0)),
      theta(params(1)),
      t_z(params(2)),
      r_x(params(3)),
      r_y(params(4)),
      r_z(params(5)) {}

JointSolver::DifferentiableTransform3d::DifferentiableTransform3d(
    frc::Pose3d pose)
    : scaler(std::hypot(pose.Translation().X().value(),
                        pose.Translation().Y().value())),
      theta(std::atan2(pose.Translation().Y().value(),
                       pose.Translation().X().value())),
      t_z(pose.Translation().Z().value()),
      r_x(pose.Rotation().X().value()),
      r_y(pose.Rotation().Y().value()),
      r_z(pose.Rotation().Z().value()) {}

JointSolver::DifferentiableTransform3d::DifferentiableTransform3d(
    frc::Transform3d pose)
    : scaler(std::hypot(pose.Translation().X().value(),
                        pose.Translation().Y().value())),
      theta(std::atan2(pose.Translation().Y().value(),
                       pose.Translation().X().value())),
      t_z(pose.Translation().Z().value()),
      r_x(pose.Rotation().X().value()),
      r_y(pose.Rotation().Y().value()),
      r_z(pose.Rotation().Z().value()) {}

JointSolver::DifferentiableTransform3d::DifferentiableTransform3d(
    Eigen::Matrix4d matrix)
    : scaler(std::hypot(matrix(0, 3), matrix(1, 3))),
      theta(std::atan2(matrix(1, 3), matrix(0, 3))),
      t_z(matrix(2, 3)) {
  Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d euler = R.canonicalEulerAngles(2, 1, 0);
  r_x = euler(2);
  r_z = euler(0);
  r_y = euler(1);
}

void JointSolver::DifferentiableTransform3d::Update(
    transform3d_derivative_t derrivative, double lr_translation,
    double lr_rotation) {
  scaler -= derrivative.scaler * lr_translation;
  theta -= derrivative.theta * lr_translation;
  // t_z -= derrivative.t_z * lr_translation;

  // r_x -= derrivative.r_x * lr;
  // r_y -= derrivative.r_y * lr;
  r_z -= derrivative.r_z * lr_rotation;
}

void JointSolver::DifferentiableTransform3d::RegisterInputs(tape_type& tape) {
  tape.registerInput(r_x);
  tape.registerInput(r_y);
  tape.registerInput(r_z);

  tape.registerInput(scaler);
  tape.registerInput(theta);
  tape.registerInput(t_z);
}

void JointSolver::DifferentiableTransform3d::RegisterOutputs(tape_type& tape) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      tape.registerOutput(matrix[i][j]);
    }
  }
}

auto JointSolver::DifferentiableTransform3d::ToEigen()
    -> Eigen::Matrix4d const {
  // clang-format off
  return (Eigen::Matrix<double, 4, 4>() << 
  matrix[0][0].value(), matrix[0][1].value(), matrix[0][2].value(), matrix[0][3].value(),
  matrix[1][0].value(), matrix[1][1].value(), matrix[1][2].value(), matrix[1][3].value(),
  matrix[2][0].value(), matrix[2][1].value(), matrix[2][2].value(), matrix[2][3].value(),
  matrix[3][0].value(), matrix[3][1].value(), matrix[3][2].value(), matrix[3][3].value())
  .finished();
  // clang-format on
}

void JointSolver::DifferentiableTransform3d::CalculateMatrix() {
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
  matrix[0][3] = std::cos(theta) * scaler;
  matrix[1][3] = std::sin(theta) * scaler;
  matrix[2][3] = t_z;

  matrix[3][0] = 0;
  matrix[3][1] = 0;
  matrix[3][3] = 1;
  matrix[3][2] = 0;
}

auto JointSolver::DifferentiableTransform3d::BackPropagate(
    const Eigen::Matrix4d& next_derrivative, tape_type& tape)
    -> transform3d_derivative_t {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      tape.registerOutput(matrix[i][j]);
      derivative(matrix[i][j]) = next_derrivative(i, j);
    }
  }
  tape.computeAdjoints();
  transform3d_derivative_t derrivative{
      .scaler = xad::derivative(scaler),
      .theta = xad::derivative(theta),
      .t_z = xad::derivative(t_z),
      .r_x = xad::derivative(r_x),
      .r_y = xad::derivative(r_y),
      .r_z = xad::derivative(r_z),
  };
  return derrivative;
}

auto JointSolver::CalculateDerivative(const Eigen::Matrix4d& robot_to_feild,
                                      const Eigen::Matrix4d& feild_to_tag,
                                      const Eigen::Matrix4d& camera_to_robot,
                                      const Eigen::Matrix3d& camera_matrix,
                                      const Eigen::Vector3d& image_point,
                                      int corner_index) -> Eigen::Matrix4d {
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

auto JointSolver::CalculateLoss(const Eigen::Matrix4d& robot_to_feild,
                                const Eigen::Matrix4d& feild_to_tag,
                                const Eigen::Matrix4d& camera_to_robot,
                                const Eigen::Matrix3d& camera_matrix,
                                const Eigen::Vector3d& image_point,
                                int corner_index) -> double {

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_robot * robot_to_feild * feild_to_tag *
      rotate_yaw *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  return normalized_points_d.array().square().sum();
}

auto JointSolver::NormalizePoint(
    const cv::Point2d& image_point,
    const camera::camera_constant_t& camera_constant) -> Eigen::Vector3d {
  // clang-format off
    return (Eigen::Vector3d() << 
      1,
      image_point.x / camera_constant.frame_width.value(),
      image_point.y / camera_constant.frame_height.value())
   .finished();
  // clang-format on
}

auto JointSolver::ProjectPoints(const Eigen::MatrixXd& A,
                                const Eigen::MatrixXd& correction,
                                const Eigen::Vector4d& x) -> Eigen::Vector3d {

  Eigen::Vector3d projected_point = A * correction * x;
  auto normalized_point = projected_point / projected_point[0];
  return normalized_point;
}

auto JointSolver::NormalizeCameraMatrix(
    Eigen::Matrix3d camera_matrix,
    const camera::camera_constant_t& camera_constant) -> Eigen::Matrix3d {
  camera_matrix(1, 0) /= camera_constant.frame_width.value();

  camera_matrix(2, 0) /= camera_constant.frame_height.value();
  camera_matrix(2, 2) /= camera_constant.frame_height.value();
  camera_matrix(1, 1) /= camera_constant.frame_width.value();
  return camera_matrix;
}

auto JointSolver::CreateTransformationMatrix(const Eigen::VectorXd& params)
    -> Eigen::Matrix4d {
  double tx = params[0];
  double ty = params[1];
  double tz = params[2];

  double rx = params[3];
  double ry = params[4];
  double rz = params[5];

  const double cos_x = std::cos(rx);
  const double cos_y = std::cos(ry);
  const double cos_z = std::cos(rz);

  const double sin_x = std::sin(rx);
  const double sin_y = std::sin(ry);
  const double sin_z = std::sin(rz);

  // clang-format off
  return (Eigen::Matrix<double, 4, 4>() <<
      cos_z * cos_y, cos_z * sin_y * sin_x - sin_z * cos_x, cos_z * sin_y * cos_x + sin_z * sin_x, tx,
      sin_z * cos_y, sin_z * sin_y * sin_x + cos_z * cos_x, sin_z * sin_y * cos_x - cos_z * sin_x, ty,
      -sin_y,        cos_y * sin_x,                         cos_y * cos_x,                         tz,
      0,             0,                                     0,                                     1)
      .finished();
  // clang-format on
}

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants,
    const AprilTagFieldLayout& layout)
    : camera_constant_(camera_constants) {
  for (size_t i = 0; i < camera_constants.size(); i++) {
    camera_name_to_index[camera_constants[i].name] = i;

    normalized_camera_matrix_.emplace_back(NormalizeCameraMatrix(
        CameraMatrixFromJson<Eigen::Matrix3d>(
            ReadIntrinsics(camera_constants[i].intrinsics_path.value())),
        camera_constants[i]));

    robot_to_camera_.emplace_back(
        ExtrinsicsJsonToCameraToRobot(
            ReadExtrinsics(camera_constants[i].extrinsics_path.value()))
            .ToMatrix()
            .inverse());
  }
}

auto JointSolver::CalculateResidual(const Eigen::VectorXd& params)
    -> Eigen::MatrixXd {
  const Eigen::MatrixXd correction = CreateTransformationMatrix(params);
  Eigen::MatrixXd residual(data_points_.size(), 3);
  for (size_t i = 0; i < data_points_.size(); i++) {
    const auto& data_point = data_points_[i];
    auto projected_point =
        ProjectPoints(data_point.A, correction, data_point.x);
    const auto point_residual =
        projected_point - data_point.normalized_image_point;
    residual(i, 0) = point_residual[0];
    residual(i, 1) = point_residual[1];
    residual(i, 2) = point_residual[2];
  }
  LOG(INFO) << "residual\n" << residual;
  return residual;
}

auto JointSolver::CalculateJacobian(const Eigen::VectorXd& params)
    -> Eigen::MatrixXd {

  const Eigen::Matrix4d correction = CreateTransformationMatrix(params);

  for (size_t i = 0; i < data_points_.size(); i++) {
    const auto& data_point = data_points_[i];

    Eigen::Vector3d projected_point =
        data_points_[i].A * correction * data_points_[i].x;
    auto normalized_point = projected_point / projected_point[0];

    // [z u v]
    const double z = projected_point[0];
    const double u = projected_point[1];
    const double v = projected_point[2];

    // clang-format off
    const Eigen::Matrix<double, 3, 3> normalized_point_d =
      (Eigen::Matrix<double, 3, 3>() << 
      0, 0, 0,
      -u / square(z), 1/z, 0,
      -v / square(z), 0, 1/z).finished();
    // clang-format on

    const Eigen::MatrixXd A_d = normalized_point_d * data_points_[i].A;

    Eigen::Matrix<double, 3, 6> J;
    J(0, 0) = 0;  // dz/t_x
    J(0, 1) = 0;  // dz/t_y
    J(0, 2) = 0;  // dz/t_z
    J(0, 3) = 0;  // dz/r_x
    J(0, 4) = 0;  // dz/r_y
    J(0, 5) = 0;  // dz/r_z

    J(1, 0) = A_d(1, 0) * data_points_[i].x(3);  // du/t_x
    J(0, 1) = A_d(1, 1) * data_points_[i].x(3);  // du/t_y
    J(0, 2) = A_d(1, 2) * data_points_[i].x(3);  // du/t_z
    J(0, 3) = 0;                                 // du/r_x
    J(0, 4) = 0;                                 // du/r_y
    J(0, 5) = 0;                                 // du/r_z

    LOG(INFO) << "A_d\n" << A_d;
  }
  return {};
}

auto JointSolver::EstimatePosition(
    const std::map<std::string, std::vector<tag_detection_t>>&
        camera_detections,
    std::optional<frc::Pose3d> initial_pose) -> position_estimate_t {
  std::vector<int> tag_ids;
  double average_timestamp = 0;
  int total_detections = 0;
  field_to_robot_ = initial_pose.value_or(position_receiver_.Get()).ToMatrix();
  const auto robot_to_feild = field_to_robot_.inverse();

  data_points_.clear();

  for (const auto& [camera_name, tag_detections] : camera_detections) {
    int index = camera_name_to_index[camera_name];
    for (const auto& tag_detection : tag_detections) {
      tag_ids.push_back(tag_detection.tag_id);
      average_timestamp += tag_detection.timestamp;
      total_detections++;
      int corner_index = 0;
      for (const auto& corner : tag_detection.corners) {
        const auto field_to_tag =
            kapriltag_layout.GetTagPose(tag_detection.tag_id)->ToMatrix();
        const auto homogenized_apriltag_corner =
            localization::kapriltag_corners_eigen_homogenized[corner_index];
        const auto normalized_camera_matrix = normalized_camera_matrix_[index];
        const auto camera_to_robot = robot_to_camera_[index].inverse();
        data_points_.push_back(datapoint_t{
            .normalized_image_point =
                NormalizePoint(corner, camera_constant_[index]),
            .normalized_camera_matrix = normalized_camera_matrix,
            .camera_to_robot = camera_to_robot,
            .feild_to_tag = field_to_tag,
            .homogenized_apriltag_corner = homogenized_apriltag_corner,
            .x = robot_to_feild * field_to_tag * rotate_yaw *
                 homogenized_apriltag_corner,
            .A = normalized_camera_matrix * PI * camera_to_robot});
        corner_index++;
      }
    }
  }

  const Eigen::VectorXd kZeroTransformParams = Eigen::VectorXd::Zero(6);
  CalculateResidual(kZeroTransformParams);
  CalculateJacobian(kZeroTransformParams);

  return position_estimate_t{
      .tag_ids = tag_ids,
      .rejected_tag_ids = {},
      .pose = {},
      .variance = 1,
      .timestamp = average_timestamp,
      .num_tags = total_detections,
      .avg_tag_dist = -1,
      .latency = -1,
      .invalid = false,
      .loss = 0,
  };
}

auto operator<<(std::ostream& os, const JointSolver::Transfrom3dDerivative& d)
    -> std::ostream& {
  os << "scaler=" << d.scaler << "\n"
     << "theta=" << d.theta << "\n"
     << "t_z=" << d.t_z << "\n"
     << "r_x=" << d.r_x << "\n"
     << "r_y=" << d.r_y << "\n"
     << "r_z=" << d.r_z << "";
  return os;
}

auto operator<<(std::ostream& os,
                const JointSolver::DifferentiableTransform3d& t)
    -> std::ostream& {
  os << "scaler=" << t.scaler << "\n"
     << "theta=" << t.theta << "\n"
     << "t_z=" << t.t_z << "\n"
     << "r_x=" << t.r_x << "\n"
     << "r_y=" << t.r_y << "\n"
     << "r_z=" << t.r_z << "";
  return os;
}

}  // namespace localization
