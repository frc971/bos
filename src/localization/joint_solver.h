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
  using mode = xad::adj<double, 2>;
  using tape_type = mode::tape_type;
  using AD = mode::active_type;

  using datapoint_t = struct DataPoint {
    Eigen::Vector3d normalized_image_point;
    Eigen::Matrix3d normalized_camera_matrix;
    Eigen::Matrix4d camera_to_robot;
    Eigen::Matrix4d feild_to_tag;
    Eigen::Vector4d homogenized_apriltag_corner;
    Eigen::Vector4d x;
    Eigen::MatrixXd A;
  };

  using differentiable_transform3d_t = struct DifferentiableTransform3d {
    AD t_x = 0;
    AD t_y = 0;
    AD t_z = 0;
    AD r_x = 0;
    AD r_y = 0;
    AD r_z = 0;

    auto ToEigen() -> Eigen::Matrix4d;
    auto ToMatrix() -> std::array<std::array<AD, 4>, 4>;
    void Update(const Eigen::VectorXd& update);
  };

 public:
  auto static CalculateLoss(const Eigen::Matrix4d& robot_to_feild,
                            const Eigen::Matrix4d& feild_to_tag,
                            const Eigen::Matrix4d& camera_to_robot,
                            const Eigen::Matrix3d& camera_matrix,
                            const Eigen::Vector3d& image_point,
                            int corner_index) -> double;

  auto static NormalizePoint(const cv::Point2d& image_point,
                             const camera::camera_constant_t& camera_constant,
                             const cv::Mat& camera_matrix,
                             const cv::Mat& distortion_coefficients)
      -> Eigen::Vector3d;

  auto static ProjectPoints(const Eigen::MatrixXd& A,
                            const Eigen::MatrixXd& correction,
                            const Eigen::Vector4d& x) -> Eigen::Vector3d;

  auto static NormalizeCameraMatrix(
      Eigen::Matrix3d camera_matrix,
      const camera::camera_constant_t& camera_constant) -> Eigen::Matrix3d;

  auto static CreateTransformationMatrix(const Eigen::VectorXd& params)
      -> Eigen::Matrix4d;

  auto static Multiply(const std::array<std::array<AD, 4>, 4>& a,
                       const Eigen::Vector4d& b) -> std::array<AD, 4>;

  auto static Multiply(const std::array<std::array<AD, 4>, 4>& a,
                       const Eigen::Matrix4d& b) -> Eigen::Matrix4d;

  auto static Multiply(const Eigen::Matrix<double, 3, 4>& a,
                       const std::array<AD, 4>& b) -> std::array<AD, 3>;

  // Jacobian should be data_points_.size() x 6(number of params in correction)
  // index is should be from 1...data_points_.size()
  void static SaveJacobian(Eigen::MatrixXd& J,
                           const differentiable_transform3d_t& correction,
                           int index);

  void static SaveResidual(Eigen::VectorXd& residual, double u_residual,
                           double v_residual, int index);

  auto static CalculateUpdate(const Eigen::MatrixXd& J,
                              const Eigen::VectorXd& residual, double lambda)
      -> Eigen::VectorXd;

  auto static CalculateResidualLoss(
      const differentiable_transform3d_t& correction,
      const std::vector<datapoint_t>& data_points) -> double;

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
  std::vector<cv::Mat> camera_matrix_;
  std::vector<cv::Mat> distortion_coefficients_;
  std::vector<Eigen::Matrix3d> normalized_camera_matrix_;
  std::vector<Eigen::Matrix4d> robot_to_camera_;
  std::vector<camera::camera_constant_t> camera_constants_;

  std::vector<datapoint_t> data_points_;

  PositionReceiver position_receiver_;
  tape_type tape_;
};

auto operator<<(std::ostream& os,
                const JointSolver::DifferentiableTransform3d& t)
    -> std::ostream&;
}  // namespace localization
//
