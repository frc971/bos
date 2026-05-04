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

  using datapoint_t = struct DataPoint {
    const Eigen::Vector3d& normalized_image_point;
    const Eigen::Matrix3d& normalized_camera_matrix;
    const Eigen::Matrix4d& camera_to_robot;
    const Eigen::Matrix4d& feild_to_tag;
    const Eigen::Vector4d& homogenized_apriltag_corner;
  };

  using transform3d_derivative_t = struct Transfrom3dDerivative {
    double scaler = 0;
    double theta = 0;
    double t_z = 0;
    double r_x = 0;
    double r_y = 0;
    double r_z = 0;

    auto operator+(const Transfrom3dDerivative other) -> Transfrom3dDerivative;
    auto operator-(const Transfrom3dDerivative other) -> Transfrom3dDerivative;
    auto operator*(const double other) -> Transfrom3dDerivative;
    auto operator+(const double other) -> Transfrom3dDerivative;
    auto operator*(const Transfrom3dDerivative other) -> Transfrom3dDerivative;
    auto operator/(const Transfrom3dDerivative other) -> Transfrom3dDerivative;
    auto sqrt() -> Transfrom3dDerivative;
  };

  struct DifferentiableTransform3d {
    AD scaler;
    AD theta;

    AD t_z;
    AD r_x;
    AD r_y;
    AD r_z;
    std::array<std::array<AD, 4>, 4> matrix;

    DifferentiableTransform3d(frc::Pose3d pose);
    DifferentiableTransform3d(frc::Transform3d pose);
    DifferentiableTransform3d(Eigen::Matrix4d matrix);
    DifferentiableTransform3d(const Eigen::VectorXd& params);

    void Update(transform3d_derivative_t derrivative, double lr_translation,
                double lr_rotation);
    void RegisterInputs(tape_type& tape);
    void RegisterOutputs(tape_type& tape);
    auto ToEigen() -> Eigen::Matrix4d const;
    void CalculateMatrix();
    auto BackPropagate(const Eigen::Matrix4d& next_derrivative, tape_type& tape)
        -> transform3d_derivative_t;
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

  auto static ProjectPoints(const Eigen::Matrix4d feild_to_robot,
                            const Eigen::Matrix4d feild_to_tag,
                            const Eigen::Matrix3d& camera_matrix,
                            const Eigen::Matrix4d& camera_to_robot,
                            const Eigen::Vector4d& homogenized_apriltag_corner)
      -> Eigen::Vector3d;

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
  auto CalculateResidual(const Eigen::VectorXd& candidate) -> Eigen::VectorXd;

 private:
  std::unordered_map<std::string, int> camera_name_to_index;
  std::vector<Eigen::Matrix3d> normalized_camera_matrix_;
  std::vector<Eigen::Matrix4d> robot_to_camera_;
  std::vector<camera::camera_constant_t> camera_constant_;

  std::vector<datapoint_t> data_points_;
  Eigen::Matrix4d field_to_robot_;

  PositionReceiver position_receiver_;
  tape_type tape_;
};

auto operator<<(std::ostream& os, const JointSolver::Transfrom3dDerivative& d)
    -> std::ostream&;

auto operator<<(std::ostream& os,
                const JointSolver::DifferentiableTransform3d& t)
    -> std::ostream&;
}  // namespace localization
//
