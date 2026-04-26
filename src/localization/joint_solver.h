#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
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
    double t_x = 0;
    double t_y = 0;
    double t_z = 0;
    double r_x = 0;
    double r_y = 0;
    double r_z = 0;

    auto operator+(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.t_x = t_x + other.t_x,
                                    .t_y = t_y + other.t_y,
                                    .t_z = t_z + other.t_z,
                                    .r_x = r_x + other.r_x,
                                    .r_y = r_y + other.r_y,
                                    .r_z = r_z + other.r_z};
    }

    auto operator-(const Transfrom3dDerrivative other)
        -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.t_x = t_x - other.t_x,
                                    .t_y = t_y - other.t_y,
                                    .t_z = t_z - other.t_z,
                                    .r_x = r_x - other.r_x,
                                    .r_y = r_y - other.r_y,
                                    .r_z = r_z - other.r_z};
    }

    auto operator*(const double other) -> Transfrom3dDerrivative {
      return Transfrom3dDerrivative{.t_x = t_x * other,
                                    .t_y = t_y * other,
                                    .t_z = t_z * other,
                                    .r_x = r_x * other,
                                    .r_y = r_y * other,
                                    .r_z = r_z * other};
    }
  };

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

    void Update(transform3d_derrivative_t derrivative, double lr) {
      t_x -= derrivative.t_x * lr;
      t_y -= derrivative.t_y * lr;
      t_z -= derrivative.t_z * lr;

      r_x -= derrivative.r_x * lr;
      r_y -= derrivative.r_y * lr;
      r_z -= derrivative.r_z * lr;
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
    void Apply(const transform3d_derrivative_t& derrivative, double lr) {
      t_x -= derrivative.t_x * lr;
      t_y -= derrivative.t_y * lr;
      t_z -= derrivative.t_z * lr;

      r_x -= derrivative.r_x * lr;
      r_y -= derrivative.r_y * lr;
      r_z -= derrivative.r_z * lr;
    }
  };

 public:
  JointSolver(const std::vector<camera::camera_constant_t>& camera_constants,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto EstimatePosition(
      const std::map<std::string, std::vector<tag_detection_t>>&
          camera_detections,
      const frc::Pose3d& starting_pose) -> position_estimate_t;

 private:
  std::unordered_map<std::string, int> camera_name_to_index;
  std::vector<Eigen::Matrix3d> normalized_camera_matrix_;
  std::vector<Eigen::Matrix4d> camera_to_robot_;
};
}  // namespace localization
