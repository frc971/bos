#include "src/localization/joint_solver.h"
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

#define EPOCHS 200

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

auto JointSolver::DifferentiableTransform3d::ToMatrix()
    -> std::array<std::array<AD, 4>, 4> {
  AD cos_x = xad::cos(r_x);
  AD cos_y = xad::cos(r_y);
  AD cos_z = xad::cos(r_z);

  AD sin_x = xad::sin(r_x);
  AD sin_y = xad::sin(r_y);
  AD sin_z = xad::sin(r_z);

  // Rotation
  std::array<std::array<AD, 4>, 4> matrix;
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
  matrix[3][3] = 1;
  matrix[3][2] = 0;
  return matrix;
}
void JointSolver::DifferentiableTransform3d::Update(
    const Eigen::VectorXd& update) {
  CHECK(update.size() == 6);
  t_x += update[0];
  t_y += update[1];
  t_z += update[2];

  r_x += update[3];
  r_y += update[4];
  r_z += update[5];
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
auto JointSolver::Multiply(const std::array<std::array<AD, 4>, 4>& a,
                           const Eigen::Vector4d& b) -> std::array<AD, 4> {
  std::array<AD, 4> out{};
  for (int i = 0; i < 4; i++) {
    out[i] = 0;
    for (int j = 0; j < 4; j++) {
      out[i] += a[i][j] * b[j];
    }
  }
  return out;
}
auto JointSolver::Multiply(const Eigen::Matrix<double, 3, 4>& a,
                           const std::array<AD, 4>& b) -> std::array<AD, 3> {
  std::array<AD, 3> out{};
  for (int i = 0; i < 3; i++) {
    out[i] = 0;
    for (int j = 0; j < 4; j++) {
      out[i] += a(i, j) * b[j];
    }
  }
  return out;
}

void JointSolver::SaveJacobian(Eigen::MatrixXd& J,
                               const differentiable_transform3d_t& correction,
                               int index) {
  const int offset = index * 2;
  J(0 + offset, 0) = correction.t_x.getAdjoint()[0];
  J(0 + offset, 1) = correction.t_y.getAdjoint()[0];
  J(0 + offset, 2) = correction.t_z.getAdjoint()[0];
  J(0 + offset, 3) = correction.r_x.getAdjoint()[0];
  J(0 + offset, 4) = correction.r_y.getAdjoint()[0];
  J(0 + offset, 5) = correction.r_z.getAdjoint()[0];
  J(1 + offset, 0) = correction.t_x.getAdjoint()[1];
  J(1 + offset, 1) = correction.t_y.getAdjoint()[1];
  J(1 + offset, 2) = correction.t_z.getAdjoint()[1];
  J(1 + offset, 3) = correction.r_x.getAdjoint()[1];
  J(1 + offset, 4) = correction.r_y.getAdjoint()[1];
  J(1 + offset, 5) = correction.r_z.getAdjoint()[1];
}

void JointSolver::SaveResidual(Eigen::VectorXd& residual, double u_residual,
                               double v_residual, int index) {
  const int offset = index * 2;
  residual[0 + offset] = u_residual;
  residual[1 + offset] = v_residual;
}

auto JointSolver::CalculateUpdate(const Eigen::MatrixXd& J,
                                  const Eigen::VectorXd& residual,
                                  double lambda) -> Eigen::VectorXd {
  constexpr double kEpsilon = 1e-9;

  const Eigen::MatrixXd j_t_j = J.transpose() * J;
  Eigen::VectorXd diagonal = j_t_j.diagonal();
  for (int i = 0; i < diagonal.size(); ++i) {
    if (diagonal[i] <= kEpsilon) {
      diagonal[i] = kEpsilon;
    }
  }

  const Eigen::MatrixXd damping =
      (lambda * diagonal).asDiagonal().toDenseMatrix();
  const Eigen::MatrixXd damped_hessian =
      j_t_j + damping +
      kEpsilon * Eigen::MatrixXd::Identity(J.cols(), J.cols());
  const Eigen::VectorXd gradient = J.transpose() * residual;

  const Eigen::LDLT<Eigen::MatrixXd> solver(damped_hessian);
  if (solver.info() != Eigen::Success) {
    return Eigen::VectorXd::Zero(J.cols());
  }

  return solver.solve(-gradient);
}

auto JointSolver::CalculateResidualLoss(
    const differentiable_transform3d_t& correction,
    const std::vector<datapoint_t>& data_points) -> double {
  Eigen::VectorXd params(6);
  params << value(correction.t_x), value(correction.t_y), value(correction.t_z),
      value(correction.r_x), value(correction.r_y), value(correction.r_z);

  const Eigen::Matrix4d correction_matrix = CreateTransformationMatrix(params);

  double loss = 0.0;
  for (const auto& data_point : data_points) {
    const Eigen::Vector3d projected_point =
        ProjectPoints(data_point.A, correction_matrix, data_point.x);
    const double u_residual =
        projected_point[1] - data_point.normalized_image_point[1];
    const double v_residual =
        projected_point[2] - data_point.normalized_image_point[2];
    loss += u_residual * u_residual + v_residual * v_residual;
  }

  return loss;
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

    // LOG(INFO) << "A_d\n" << A_d;
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

  differentiable_transform3d_t correction;
  value(correction.t_x) = 0.5;
  value(correction.t_y) = -0.5;
  value(correction.t_z) = 0.3;
  value(correction.r_x) = 0.3;
  value(correction.r_y) = -0.3;
  value(correction.r_z) = 0.4;

  double checkpoint_loss = CalculateResidualLoss(correction, data_points_);
  double lambda = 1e-4;
  constexpr double kMinLambda = 1e-12;
  constexpr double kMaxLambda = 1e12;
  constexpr double kMinUpdateNorm = 1e-14;
  constexpr double kMinLossImprovement = 1e-15;
  constexpr int kMaxAttemptsPerEpoch = 10;
  constexpr int kMaxStaleEpochs = 20;
  int stale_epochs = 0;

  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    Eigen::MatrixXd J(data_points_.size() * 2, 6);
    Eigen::VectorXd residual(data_points_.size() * 2);

    tape_.registerInput(correction.t_x);
    tape_.registerInput(correction.t_y);
    tape_.registerInput(correction.t_z);
    tape_.registerInput(correction.r_x);
    tape_.registerInput(correction.r_y);
    tape_.registerInput(correction.r_z);
    tape_.newRecording();
    int index = 0;
    for (auto const& data_point : data_points_) {
      const auto correction_matrix = correction.ToMatrix();
      auto a = Multiply(correction_matrix, data_point.x);
      std::array<AD, 3> projected_point = Multiply(data_point.A, a);
      AD u = projected_point[1] / projected_point[0];
      AD v = projected_point[2] / projected_point[0];
      tape_.registerOutput(u);
      tape_.registerOutput(v);
      u.setAdjoint({1.0, 0.0});
      v.setAdjoint({0.0, 1.0});
      tape_.computeAdjoints();
      SaveJacobian(J, correction, index);
      SaveResidual(residual, u.value() - data_point.normalized_image_point[1],
                   v.value() - data_point.normalized_image_point[2], index);

      index++;
    }
    const double loss = residual.squaredNorm();
    bool accepted = false;
    double best_candidate_loss = checkpoint_loss;
    Eigen::VectorXd accepted_update = Eigen::VectorXd::Zero(6);
    double trial_lambda = lambda;

    for (int attempt = 0; attempt < kMaxAttemptsPerEpoch; ++attempt) {
      const Eigen::VectorXd update = CalculateUpdate(J, residual, trial_lambda);
      if (update.norm() < kMinUpdateNorm) {
        break;
      }

      differentiable_transform3d_t candidate = correction;
      candidate.Update(update);
      const double candidate_loss =
          CalculateResidualLoss(candidate, data_points_);

      if (candidate_loss < checkpoint_loss) {
        accepted = true;
        accepted_update = update;
        best_candidate_loss = candidate_loss;
        lambda = std::max(kMinLambda, trial_lambda * 0.25);
        break;
      }

      trial_lambda = std::min(kMaxLambda, trial_lambda * 8.0);
    }

    LOG(INFO) << "--------------------------------------";
    // LOG(INFO) << "J\n" << J;
    LOG(INFO) << "Residual\n" << residual;
    LOG(INFO) << "Loss: " << loss;
    LOG(INFO) << "Checkpoint Loss: " << checkpoint_loss;
    LOG(INFO) << "Candidate Loss: " << best_candidate_loss;
    LOG(INFO) << "lambda: " << lambda;

    if (accepted) {
      const double loss_improvement = checkpoint_loss - best_candidate_loss;
      correction.Update(accepted_update);
      checkpoint_loss = best_candidate_loss;
      if (loss_improvement < kMinLossImprovement) {
        stale_epochs++;
      } else {
        stale_epochs = 0;
      }
    } else {
      stale_epochs++;
      lambda = std::min(kMaxLambda, trial_lambda);
      if (lambda >= kMaxLambda) {
        break;
      }
    }

    if (stale_epochs >= kMaxStaleEpochs) {
      break;
    }
  }

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

auto operator<<(std::ostream& os,
                const JointSolver::differentiable_transform3d_t& t)
    -> std::ostream& {
  os << "t_x=" << t.t_x << "\n"
     << "t_y=" << t.t_y << "\n"
     << "t_z=" << t.t_z << "\n"
     << "r_x=" << t.r_x << "\n"
     << "r_y=" << t.r_y << "\n"
     << "r_z=" << t.r_z << "";
  return os;
}

}  // namespace localization
