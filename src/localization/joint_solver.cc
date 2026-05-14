#include "src/localization/joint_solver.h"
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

using frc::AprilTagFieldLayout;
using utils::CameraMatrixFromJson;
using utils::DistortionCoefficientsFromJson;
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
auto JointSolver::DifferentiableTransform3d::ToEigen() -> Eigen::Matrix4d {
  const auto matrix = ToMatrix();
  Eigen::Matrix4d out = Eigen::Matrix4d::Zero();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      out(i, j) = value(matrix[i][j]);
    }
  }
  return out;
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

auto UndistortPixelPoint(const cv::Point2f& distortedPoint,
                         const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
    -> cv::Point2f {
  std::vector<cv::Point2f> src = {distortedPoint};
  std::vector<cv::Point2f> dst;
  cv::undistortPoints(src, dst, cameraMatrix, distCoeffs, cv::noArray(),
                      cameraMatrix);
  return dst[0];
}

auto NormalizePoint(const cv::Point2d& image_point,
                    const camera::camera_constant_t& camera_constant,
                    const cv::Mat& camera_matrix,
                    const cv::Mat& distortion_coefficients) -> Eigen::Vector3d {
  cv::Point2f undistorted_point =
      UndistortPixelPoint(image_point, camera_matrix, distortion_coefficients);
  // clang-format off
    return (Eigen::Vector3d() << 
      1,
      undistorted_point.x / camera_constant.frame_width.value(),
      undistorted_point.y / camera_constant.frame_height.value())
   .finished();
  // clang-format on
}

auto ProjectPoints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& correction,
                   const Eigen::Vector4d& x) -> Eigen::Vector3d {

  Eigen::Vector3d projected_point = A * correction * x;
  auto normalized_point = projected_point / projected_point[0];
  return normalized_point;
}

auto NormalizeCameraMatrix(Eigen::Matrix3d camera_matrix,
                           const camera::camera_constant_t& camera_constant)
    -> Eigen::Matrix3d {
  camera_matrix(1, 0) /= camera_constant.frame_width.value();

  camera_matrix(2, 0) /= camera_constant.frame_height.value();
  camera_matrix(2, 2) /= camera_constant.frame_height.value();
  camera_matrix(1, 1) /= camera_constant.frame_width.value();
  return camera_matrix;
}

auto CreateTransformationMatrix(const Eigen::VectorXd& params)
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
                           const Eigen::Matrix4d& b) -> Eigen::Matrix4d {
  Eigen::Matrix4d out = Eigen::Matrix4d::Zero();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        out(i, j) += value(a[i][k]) * b(k, j);
      }
    }
  }
  return out;
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
  constexpr double kEpsilon = 1e-12;

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

  const double residual_count =
      static_cast<double>(std::max<size_t>(1, data_points.size() * 2));
  return loss / residual_count;
}

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants,
    AprilTagFieldLayout layout)
    : layout_(std::move(layout)), camera_constants_(camera_constants) {
  for (size_t i = 0; i < camera_constants.size(); i++) {
    camera_name_to_index[camera_constants[i].name] = i;

    distortion_coefficients_.emplace_back(
        DistortionCoefficientsFromJson<cv::Mat>(
            ReadIntrinsics(camera_constants[i].intrinsics_path.value())));

    camera_matrix_.emplace_back(CameraMatrixFromJson<cv::Mat>(
        ReadIntrinsics(camera_constants[i].intrinsics_path.value())));

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

auto JointSolver::EstimatePosition(
    const std::map<std::string, std::vector<tag_detection_t>>&
        camera_detections,
    std::optional<frc::Pose3d> initial_pose) -> position_estimate_t {
  std::vector<int> tag_ids;
  double average_timestamp = 0;
  int total_detections = 0;
  const Eigen::Matrix4d field_to_robot =
      initial_pose.value_or(frc::Pose3d(position_receiver_.Get())).ToMatrix();
  const Eigen::Matrix4d robot_to_feild = field_to_robot.inverse();

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
            .normalized_image_point = NormalizePoint(
                corner, camera_constants_[index], camera_matrix_[index],
                distortion_coefficients_[index]),
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
  average_timestamp /= total_detections;

  if (data_points_.size() < 3) {
    return position_estimate_t{
        .tag_ids = tag_ids,
        .rejected_tag_ids = {},
        .pose = {},
        .variance = 1,
        .timestamp = average_timestamp,
        .num_tags = total_detections,
        .avg_tag_dist = -1,
        .latency = -1,
        .invalid = true,
        .loss = -1,
    };
  }

  differentiable_transform3d_t correction{};

  double checkpoint_loss = CalculateResidualLoss(correction, data_points_);
  double lambda = 1e-6;
  constexpr double kMinLambda = 1e-16;
  constexpr double kMaxLambda = 1e16;
  constexpr double kMinUpdateNorm = 1e-18;
  constexpr double kMinLossImprovement = 1e-17;
  constexpr int kMaxAttemptsPerEpoch = 40;
  constexpr int kMaxStaleEpochs = 400;
  constexpr int kepochs = 100;
  int stale_epochs = 0;

  for (int epoch = 0; epoch < kepochs; epoch++) {
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
        lambda = std::max(kMinLambda, trial_lambda * 0.5);
        break;
      }

      trial_lambda = std::min(kMaxLambda, trial_lambda * 2.0);
    }

    VLOG(1) << "--------------------------------------";
    VLOG(1) << "Residual\n" << residual;
    VLOG(1) << "Loss: " << loss;
    VLOG(1) << "Checkpoint Loss: " << checkpoint_loss;
    VLOG(1) << "Candidate Loss: " << best_candidate_loss;
    VLOG(1) << "lambda: " << lambda;
    VLOG(1) << "update\n" << accepted_update;
    VLOG(1) << "correction\n" << correction;
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

  Eigen::Matrix4d tmp =
      (Multiply(correction.ToMatrix(), robot_to_feild)).inverse();
  tmp(3, 0) = 0;
  tmp(3, 1) = 0;
  tmp(3, 2) = 0;
  tmp(3, 3) = 1;
  frc::Pose3d pose(tmp);

  return position_estimate_t{
      .tag_ids = tag_ids,
      .rejected_tag_ids = {},
      .pose = pose,
      .variance = 1,
      .timestamp = average_timestamp,
      .num_tags = total_detections,
      .avg_tag_dist = -1,
      .latency = -1,
      .invalid = false,
      .loss = checkpoint_loss,
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
