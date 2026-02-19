#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

namespace localization {
using frc::AprilTagFieldLayout;

using data_point_t = struct DataPoint {
  cv::Point2d undistorted_point;
  cv::Mat A;
  cv::Mat C;
  double fx;
  double fy;
};

auto constexpr square(double value) -> double {
  return value * value;
}

JointSolver::JointSolver(const std::vector<camera::Camera>& camera_configs,
                         AprilTagFieldLayout layout, double tag_size)
    : layout_(std::move(layout)) {

  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, std::numbers::pi, 0);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  rotate_z_ = utils::MakeTransform(rvec, tvec);

  // clang-format off
  pi_ = (cv::Mat_<double>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0, 
    0, 0, 1, 0);
  // clang-format on
  camera_to_robot_.reserve(camera_configs.size());
  camera_matrix_.reserve(camera_configs.size());
  distortion_coefficients_.reserve(camera_configs.size());
  for (const camera::Camera& camera_config : camera_configs) {
    camera::camera_constant_t camera_constant =
        camera::camera_constants[camera_config];
    cv::Mat camera_to_robot =
        utils::Transform3dToCvMat(utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(camera_constant.extrinsics_path)));

    auto intrinsics = utils::ReadIntrinsics(camera_constant.intrinsics_path);

    auto camera_matrix = utils::CameraMatrixFromJson<cv::Mat>(intrinsics);
    auto distotion_coefficeints =
        utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics);

    camera_to_robot_.push_back(camera_to_robot);
    camera_matrix_.push_back(camera_matrix);
    distortion_coefficients_.push_back(distotion_coefficeints);
  }

  for (const frc::AprilTag& tag : layout_.GetTags()) {
    cv::Mat field_to_tag = utils::Pose3dToCvMat(tag.pose);
    tag_corners_[tag.ID] = {
        field_to_tag * rotate_z_ *
            utils::HomogenizePoint3d(kapriltag_corners[0]),
        field_to_tag * rotate_z_ *
            utils::HomogenizePoint3d(kapriltag_corners[1]),
        field_to_tag * rotate_z_ *
            utils::HomogenizePoint3d(kapriltag_corners[2]),
        field_to_tag * rotate_z_ *
            utils::HomogenizePoint3d(kapriltag_corners[3]),
    };
  }
}

auto JointSolver::EstimatePosition(
    const std::vector<std::vector<tag_detection_t>>& detections,
    const frc::Pose3d& intial_pose) -> std::vector<position_estimate_t> {

  int total_detections = 0;
  double average_timestamp = 0;

  cv::Mat robot_to_feild = utils::Pose3dToCvMat(intial_pose).inv();

  std::vector<data_point_t> data_points;
  for (size_t camera_idx = 0; camera_idx < detections.size(); camera_idx++) {
    for (size_t detection_idx = 0;
         detection_idx < detections[camera_idx].size(); detection_idx++) {
      total_detections++;
      average_timestamp += detections[camera_idx][detection_idx].timestamp;
      for (size_t corner_idx = 0;
           corner_idx < detections[camera_idx][detection_idx].corners.size();
           corner_idx++) {
        std::vector<cv::Point2d> distorted_point(
            {detections[camera_idx][detection_idx].corners[corner_idx]});
        std::vector<cv::Point2d> undistorted_point;
        cv::undistortImagePoints(distorted_point, undistorted_point,
                                 camera_matrix_[camera_idx],
                                 distortion_coefficients_[camera_idx]);
        const auto tag_id = detections[camera_idx][detection_idx].tag_id;
        data_point_t data_point{
            .undistorted_point = undistorted_point[0],
            .A =
                camera_matrix_[camera_idx] * pi_ * camera_to_robot_[camera_idx],
            .C = tag_corners_[tag_id].value()[corner_idx],
            .fx = camera_matrix_[camera_idx].at<double>(0, 0),
            .fy = camera_matrix_[camera_idx].at<double>(1, 1),
        };

        data_points.push_back(std::move(data_point));
      }
    }
  }

  average_timestamp /= total_detections;

  const int epochs = 1000;
  for (int epoch = 0; epoch < epochs; epoch++) {
    for (const auto& data_point : data_points) {
      cv::Mat projected_point_homogenized =
          data_point.A * robot_to_feild * data_point.C;

      cv::Point2d projected_point(
          projected_point_homogenized.at<double>(0, 0) /
              projected_point_homogenized.at<double>(2, 0),
          projected_point_homogenized.at<double>(1, 0) /
              projected_point_homogenized.at<double>(2, 0));

      cv::Point2d projected_point_d =
          projected_point - data_point.undistorted_point;

      const double lambda = projected_point_homogenized.at<double>(2, 0);
      cv::Mat projected_point_homogenized_d =
          (cv::Mat_<double>(3, 1) << projected_point_d.x / lambda,
           projected_point_d.y / lambda,
           (-projected_point.x * projected_point_d.x / square(lambda)) +
               (-projected_point.y * projected_point_d.y / square(lambda)));

      cv::Mat A_d = data_point.A.t() * projected_point_homogenized_d;

      cv::Mat robot_to_feild_d = A_d * (data_point.C).t();
      robot_to_feild_d /= 1000000;

      robot_to_feild.at<double>(0, 3) -= robot_to_feild_d.at<double>(0, 3);
      robot_to_feild.at<double>(1, 3) -= robot_to_feild_d.at<double>(1, 3);
      robot_to_feild.at<double>(2, 3) -= robot_to_feild_d.at<double>(2, 3);
    }
  }

  return {position_estimate_t{
      .pose = utils::OpencvTransformationMatrixToPose3d(robot_to_feild.inv()),
      .variance = 1.0,
      .timestamp = average_timestamp,
  }};
}

auto JointSolver::ProjectPoint(const frc::Pose3d& robot_pose, int tag_id,
                               int camera_index) -> std::vector<cv::Point2d> {
  CHECK(layout_.GetTagPose(tag_id).has_value());

  cv::Mat camera_to_robot = camera_to_robot_[camera_index];
  cv::Mat feild_to_robot = utils::Pose3dToCvMat(robot_pose);
  cv::Mat feild_to_tag =
      utils::Pose3dToCvMat(layout_.GetTagPose(tag_id).value());
  std::vector<cv::Point2d> image_points;
  image_points.reserve(4);
  cv::Mat camera_to_tag =
      camera_to_robot * feild_to_robot.inv() * feild_to_tag * rotate_z_;
  for (const auto& kapriltag_corner : kapriltag_corners) {
    cv::Mat projected_points = camera_matrix_[camera_index] * pi_ *
                               camera_to_tag *
                               utils::HomogenizePoint3d(kapriltag_corner);
    image_points.emplace_back(
        projected_points.at<double>(0, 0) / projected_points.at<double>(0, 2),
        projected_points.at<double>(0, 1) / projected_points.at<double>(0, 2));
  }
  return image_points;
}

}  // namespace localization
