#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/extrinsics_from_json.h"
#include "src/utils/intrinsics_from_json.h"

namespace localization {

SquareSolver::SquareSolver(const std::string& intrinsics_path,
                           const std::string& extrinsics_path,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : layout_(std::move(layout)),
      tag_corners_(std::move(tag_corners)),
      camera_matrix_(utils::camera_matrix_from_json<cv::Mat>(
          utils::read_intrinsics(intrinsics_path))),
      distortion_coefficients_(
          utils::distortion_coefficients_from_json<cv::Mat>(
              utils::read_intrinsics(intrinsics_path))),
      camera_to_robot_(utils::ExtrinsicsJsonToCameraToRobot(
          utils::read_extrinsics(extrinsics_path))) {}

SquareSolver::SquareSolver(camera::Camera camera_config,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : SquareSolver(camera::camera_constants[camera_config].intrinsics_path,
                   camera::camera_constants[camera_config].extrinsics_path,
                   std::move(layout), std::move(tag_corners)) {}

auto SquareSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections)
    -> std::vector<position_estimate_t> {
  std::vector<position_estimate_t> position_estimates;
  position_estimates.reserve(detections.size());
  
  for (const auto& detection : detections) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_IPPE_SQUARE);

    const double translation_x = tvec.ptr<double>()[2];
    const double translation_y = tvec.ptr<double>()[0];
    const double translation_z = tvec.ptr<double>()[1];

    const double rotation_x = rvec.ptr<double>()[2];
    const double rotation_y = rvec.ptr<double>()[0];
    const double rotation_z = rvec.ptr<double>()[1];

    auto pose = frc::Pose3d(frc::Translation3d(units::meter_t{translation_x},
                                               units::meter_t{translation_y},
                                               units::meter_t{translation_z}),
                            frc::Rotation3d(units::radian_t{rotation_x},
                                            units::radian_t{rotation_y},
                                            units::radian_t{rotation_z}));

    frc::Transform3d camera_to_tag(
        units::meter_t{pose.X()}, units::meter_t{-pose.Y()},
        units::meter_t{-pose.Z()},
        frc::Rotation3d(units::radian_t{pose.Rotation().X()},
                        units::radian_t{-pose.Rotation().Y()},
                        units::radian_t{-pose.Rotation().Z()} + 180_deg));

    frc::Transform3d tag_to_camera = camera_to_tag.Inverse();
    frc::Pose3d tag_pose = layout_.GetTagPose(detection.tag_id).value();
    frc::Pose3d camera_pose = tag_pose.TransformBy(tag_to_camera);
    frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot_);

    position_estimates.push_back({robot_pose,
                                  std::hypot(translation_x, translation_y),
                                  detection.timestamp});
  }
  return position_estimates;
}

auto SquareSolver::EstimatePositionJoint(
    const std::vector<tag_detection_t>& detections)
    -> std::optional<position_estimate_t> {
  
  if (detections.empty()) {
    return std::nullopt;
  }
  
  std::vector<frc::Pose3d> camera_poses;
  double avg_timestamp = 0.0;
  
  for (const auto& detection : detections) {
    auto maybe_tag_pose = layout_.GetTagPose(detection.tag_id);
    if (!maybe_tag_pose.has_value()) {
      LOG(WARNING) << "Got invalid tag id: " << detection.tag_id;
      continue;
    }
    
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_IPPE_SQUARE);
    
    const double translation_x = tvec.ptr<double>()[2];
    const double translation_y = tvec.ptr<double>()[0];
    const double translation_z = tvec.ptr<double>()[1];
    
    const double rotation_x = rvec.ptr<double>()[2];
    const double rotation_y = rvec.ptr<double>()[0];
    const double rotation_z = rvec.ptr<double>()[1];
    
    auto pose = frc::Pose3d(frc::Translation3d(units::meter_t{translation_x},
                                               units::meter_t{translation_y},
                                               units::meter_t{translation_z}),
                            frc::Rotation3d(units::radian_t{rotation_x},
                                            units::radian_t{rotation_y},
                                            units::radian_t{rotation_z}));
    
    frc::Transform3d camera_to_tag(
        units::meter_t{pose.X()}, units::meter_t{-pose.Y()},
        units::meter_t{-pose.Z()},
        frc::Rotation3d(units::radian_t{pose.Rotation().X()},
                        units::radian_t{-pose.Rotation().Y()},
                        units::radian_t{-pose.Rotation().Z()} + 180_deg));
    
    frc::Transform3d tag_to_camera = camera_to_tag.Inverse();
    frc::Pose3d camera_pose = maybe_tag_pose.value().TransformBy(tag_to_camera);
    
    camera_poses.push_back(camera_pose);
    avg_timestamp += detection.timestamp;
  }
  
  if (camera_poses.empty()) {
    return std::nullopt;
  }
  
  avg_timestamp /= detections.size();
  
  // Average translation
  double avg_x = 0, avg_y = 0, avg_z = 0;
  for (const auto& cp : camera_poses) {
    avg_x += cp.X().value();
    avg_y += cp.Y().value();
    avg_z += cp.Z().value();
  }
  avg_x /= camera_poses.size();
  avg_y /= camera_poses.size();
  avg_z /= camera_poses.size();
  
  // Average rotation using quaternions
  double qw = 0, qx = 0, qy = 0, qz = 0;
  for (const auto& cp : camera_poses) {
    auto q = cp.Rotation().GetQuaternion();
    if (qw * q.W() + qx * q.X() + qy * q.Y() + qz * q.Z() < 0) {
      qw -= q.W();
      qx -= q.X();
      qy -= q.Y();
      qz -= q.Z();
    } else {
      qw += q.W();
      qx += q.X();
      qy += q.Y();
      qz += q.Z();
    }
  }
  
  // Normalize averaged quaternion
  double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
  
  frc::Pose3d camera_pose(
      frc::Translation3d(units::meter_t{avg_x}, units::meter_t{avg_y}, units::meter_t{avg_z}),
      frc::Rotation3d(frc::Quaternion(qw, qx, qy, qz))
  );
  
  frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot_);
  
  // Calculate average distance to detected tags
  double total_distance = 0.0;
  for (const auto& detection : detections) {
    auto tag_pose = layout_.GetTagPose(detection.tag_id);
    if (tag_pose.has_value()) {
      double dx = camera_pose.X().value() - tag_pose->X().value();
      double dy = camera_pose.Y().value() - tag_pose->Y().value();
      double dz = camera_pose.Z().value() - tag_pose->Z().value();
      total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
  }
  double avg_distance = total_distance / camera_poses.size();
  
  return position_estimate_t{robot_pose, avg_distance, avg_timestamp};
}

}  // namespace localization