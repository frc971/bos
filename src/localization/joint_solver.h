#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_receiver.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"

using json = nlohmann::json;

namespace localization {

class JointSolver {
 public:
  using mode = xad::adj<double>;
  using tape_type = mode::tape_type;
  using AD = mode::active_type;

  using transform3d_derrivative_t = struct Transfrom3dDerrivative {
    double scaler = 0;
    double theta = 0;
    double t_z = 0;
    double r_x = 0;
    double r_y = 0;
    double r_z = 0;

    auto operator+(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler + other.scaler,
                                    .theta = theta + other.theta,
                                    .t_z = t_z + other.t_z,
                                    .r_x = r_x + other.r_x,
                                    .r_y = r_y + other.r_y,
                                    .r_z = r_z + other.r_z};
    }

    auto operator-(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler - other.scaler,
                                    .theta = theta - other.theta,
                                    .t_z = t_z - other.t_z,
                                    .r_x = r_x - other.r_x,
                                    .r_y = r_y - other.r_y,
                                    .r_z = r_z - other.r_z};
    }

    auto operator*(const double other) -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler * other,
                                    .theta = theta * other,
                                    .t_z = t_z * other,
                                    .r_x = r_x * other,
                                    .r_y = r_y * other,
                                    .r_z = r_z * other};
    }

    auto operator+(const double other) -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler + other,
                                    .theta = theta + other,
                                    .t_z = t_z + other,
                                    .r_x = r_x + other,
                                    .r_y = r_y + other,
                                    .r_z = r_z + other};
    }

    auto operator*(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler * other.scaler,
                                    .theta = theta * other.theta,
                                    .t_z = t_z * other.t_z,
                                    .r_x = r_x * other.r_x,
                                    .r_y = r_y * other.r_y,
                                    .r_z = r_z * other.r_z};
    }

    auto operator/(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = scaler / other.scaler,
                                    .theta = theta / other.theta,
                                    .t_z = t_z / other.t_z,
                                    .r_x = r_x / other.r_x,
                                    .r_y = r_y / other.r_y,
                                    .r_z = r_z / other.r_z};
    }

    auto sqrt() -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.scaler = std::sqrt(scaler),
                                    .theta = std::sqrt(theta),
                                    .t_z = std::sqrt(t_z),
                                    .r_x = std::sqrt(r_x),
                                    .r_y = std::sqrt(r_y),
                                    .r_z = std::sqrt(r_z)};
    }
  };

  struct DifferntiableTransform3d {

    // Translation in meters, rotation in radians
    AD scaler;
    AD theta;

    AD t_z;
    AD r_x;
    AD r_y;
    AD r_z;
    std::array<std::array<AD, 4>, 4> matrix;

    DifferntiableTransform3d(frc::Pose3d pose)
        : scaler(std::hypot(pose.Translation().X().value(),
                            pose.Translation().Y().value())),
          theta(std::atan2(pose.Translation().Y().value(),
                           pose.Translation().X().value())),
          t_z(pose.Translation().Z().value()),
          r_x(pose.Rotation().X().value()),
          r_y(pose.Rotation().Y().value()),
          r_z(pose.Rotation().Z().value()) {}

    DifferntiableTransform3d(frc::Transform3d pose)
        : scaler(std::hypot(pose.Translation().X().value(),
                            pose.Translation().Y().value())),
          theta(std::atan2(pose.Translation().Y().value(),
                           pose.Translation().X().value())),
          t_z(pose.Translation().Z().value()),
          r_x(pose.Rotation().X().value()),
          r_y(pose.Rotation().Y().value()),
          r_z(pose.Rotation().Z().value()) {}

    DifferntiableTransform3d(Eigen::Matrix4d matrix)
        : scaler(std::hypot(matrix(0, 3), matrix(1, 3))),
          theta(std::atan2(matrix(1, 3), matrix(0, 3))),
          t_z(matrix(2, 3)) {
      Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
      Eigen::Vector3d euler = R.canonicalEulerAngles(2, 1, 0);
      r_x = euler(2);
      r_y = euler(1);
      r_z = euler(0);
    }

    void Update(transform3d_derrivative_t derrivative, double lr_translation,
                double lr_rotation) {
      scaler -= derrivative.scaler * lr_translation;
      theta -= derrivative.theta * lr_translation;
      // t_z -= derrivative.t_z * lr_translation;

      // r_x -= derrivative.r_x * lr;
      // r_y -= derrivative.r_y * lr;
      r_z -= derrivative.r_z * lr_rotation;
    }

    void RegisterInputs(tape_type& tape) {
      tape.registerInput(r_x);
      tape.registerInput(r_y);
      tape.registerInput(r_z);

      tape.registerInput(scaler);
      tape.registerInput(theta);
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
      matrix[0][3] = std::cos(theta) * scaler;
      matrix[1][3] = std::sin(theta) * scaler;
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
          .scaler = xad::derivative(scaler),
          .theta = xad::derivative(theta),
          .t_z = xad::derivative(t_z),
          .r_x = xad::derivative(r_x),
          .r_y = xad::derivative(r_y),
          .r_z = xad::derivative(r_z),
      };
      return derrivative;
    }
  };

 public:
  auto static CalculateDerivative(const Eigen::Matrix4d& robot_to_feild,
                                  const Eigen::Matrix4d& feild_to_tag,
                                  const Eigen::Matrix4d& camera_to_robot,
                                  const Eigen::Matrix3d& camera_matrix,
                                  const Eigen::Vector3d& image_point,
                                  int corner_index) -> Eigen::Matrix4d;

  auto static CalculateLoss(const Eigen::Matrix4d& robot_to_feild,
                            const Eigen::Matrix4d& feild_to_tag,
                            const Eigen::Matrix4d& camera_to_robot,
                            const Eigen::Matrix3d& camera_matrix,
                            const Eigen::Vector3d& image_point,
                            int corner_index) -> double;

  auto static NormalizePoint(const cv::Point2d& image_point,
                             const camera::camera_constant_t& camera_constant)
      -> Eigen::Vector3d;

  auto static ProjectPoints(const frc::Pose3d& camera_pose,
                            const frc::Pose3d& tag_pose,
                            const Eigen::Matrix3d& camera_matrix,
                            const Eigen::Matrix4d& camera_to_robot,
                            int corner_index) -> Eigen::Vector3d;

  auto static NormalizeCameraMatrix(
      Eigen::Matrix3d camera_matrix,
      const camera::camera_constant_t& camera_constant) -> Eigen::Matrix3d;

 public:
  JointSolver(const std::vector<camera::camera_constant_t>& camera_constants,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto EstimatePosition(
      const std::map<std::string, std::vector<tag_detection_t>>&
          camera_detections,
      std::optional<frc::Pose3d> intial_pose = std::nullopt)
      -> position_estimate_t;

 private:
  std::unordered_map<std::string, int> camera_name_to_index;
  std::vector<Eigen::Matrix3d> normalized_camera_matrix_;
  std::vector<Eigen::Matrix4d> camera_to_robot_;
  std::vector<camera::camera_constant_t> camera_constant_;
  PositionReceiver position_receiver_;
  tape_type tape_;
};

inline auto operator<<(std::ostream& os,
                       const JointSolver::Transfrom3dDerrivative& d)
    -> std::ostream& {
  os << "scaler=" << d.scaler << "\n"
     << "theta=" << d.theta << "\n"
     << "t_z=" << d.t_z << "\n"
     << "r_x=" << d.r_x << "\n"
     << "r_y=" << d.r_y << "\n"
     << "r_z=" << d.r_z << "";
  return os;
}

inline auto operator<<(std::ostream& os,
                       const JointSolver::DifferntiableTransform3d& t)
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
//
