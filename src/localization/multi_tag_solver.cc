#include "src/localization/multi_tag_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

static const cv::Mat zero_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);

namespace localization {

auto CvMatToPoint3f(cv::Mat mat) -> cv::Point3d {
  return {mat.at<double>(0), mat.at<double>(1), mat.at<double>(2)};
}

auto HomogenizePoint3d(cv::Point3d point) -> cv::Mat {
  return (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);  // NOLINT
}

auto Transform3dToCvMat(frc::Transform3d transform) -> cv::Mat {
  frc::Pose3d opencv_pose(
      frc::Translation3d(-units::meter_t{transform.Y().value()},
                         -units::meter_t{transform.Z().value()},
                         units::meter_t{transform.X().value()}),
      frc::Rotation3d(-units::radian_t{transform.Rotation().Y()},
                      -units::radian_t{transform.Rotation().Z()},
                      units::radian_t{transform.Rotation().X()}));
  return utils::EigenToCvMat(opencv_pose.ToMatrix());
}

MultiTagSolver::MultiTagSolver(const std::string& intrinsics_path,
                               const std::string& extrinsics_path,
                               const frc::AprilTagFieldLayout& layout,
                               const std::vector<cv::Point3d>& tag_corners)
    : camera_matrix_(utils::CameraMatrixFromJson<cv::Mat>(
          utils::ReadIntrinsics(intrinsics_path))),
      distortion_coefficients_(utils::DistortionCoefficientsFromJson<cv::Mat>(
          utils::ReadIntrinsics(intrinsics_path))),
      camera_to_robot_(Transform3dToCvMat(utils::ExtrinsicsJsonToCameraToRobot(
          utils::ReadExtrinsics(extrinsics_path)))) {
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, std::numbers::pi, 0);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat rotate_z = utils::MakeTransform(rvec, tvec);

  for (const frc::AprilTag& tag : layout.GetTags()) {
    cv::Mat field_to_tag = utils::Pose3dToCvMat(tag.pose);
    tag_corners_[tag.ID] = {
        CvMatToPoint3f(field_to_tag * rotate_z *
                       HomogenizePoint3d(kapriltag_corners[0])),
        CvMatToPoint3f(field_to_tag * rotate_z *
                       HomogenizePoint3d(kapriltag_corners[1])),
        CvMatToPoint3f(field_to_tag * rotate_z *
                       HomogenizePoint3d(kapriltag_corners[2])),
        CvMatToPoint3f(field_to_tag * rotate_z *
                       HomogenizePoint3d(kapriltag_corners[3])),
    };
  }
}

MultiTagSolver::MultiTagSolver(camera::Camera camera_config,
                               const frc::AprilTagFieldLayout& layout,
                               const std::vector<cv::Point3d>& tag_corners)
    : MultiTagSolver(camera::camera_constants[camera_config].intrinsics_path,
                     camera::camera_constants[camera_config].extrinsics_path,
                     layout, tag_corners) {}

auto MultiTagSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections, bool reject_far_tags)
    -> std::vector<position_estimate_t> {
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  std::vector<int> tag_ids;
  std::vector<int> rejected_tag_ids;
  double avg_distance = 0.0;
  for (const tag_detection_t& detection : detections) {
    if (!tag_corners_[detection.tag_id].has_value()) {
      LOG(WARNING) << "Invalid tag id: " << detection.tag_id;
      continue;
    }
    if (reject_far_tags) {
      const auto& c = detection.corners;
      const double area = 0.5 * std::abs((c[0].x - c[2].x) * (c[1].y - c[3].y) -
                                         (c[1].x - c[3].x) * (c[0].y - c[2].y));
      if (area < kmin_tag_area_pixels) {
        rejected_tag_ids.push_back(detection.tag_id);
        continue;
      }
    }
    cv::Mat rvec_tag = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec_tag = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnP(kapriltag_corners, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec_tag, tvec_tag, false,
                 cv::SOLVEPNP_IPPE_SQUARE);
    if (reject_far_tags && cv::norm(tvec_tag) > 5.0) {
      rejected_tag_ids.push_back(detection.tag_id);
      continue;
    }
    avg_distance += cv::norm(tvec_tag);
    tag_ids.push_back(detection.tag_id);
    image_points.insert(image_points.end(), detection.corners.begin(),
                        detection.corners.end());
    object_points.insert(object_points.end(),
                         tag_corners_[detection.tag_id].value().begin(),
                         tag_corners_[detection.tag_id].value().end());
  }
  if (image_points.size() == 0 || object_points.size() == 0) {
    return {};
  }
  avg_distance /= tag_ids.size();
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

  try {
    cv::solvePnP(object_points, image_points, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_SQPNP);
  } catch (const std::exception& e) {
    LOG(WARNING) << "Caught solve pnp exception:\n" << e.what();
    return {};
  }

  cv::Mat feild_to_camera = utils::MakeTransform(rvec, tvec).inv();
  cv::Mat feild_to_robot = feild_to_camera * camera_to_robot_;

  int num_tags = tag_ids.size();
  return {position_estimate_t{
      .tag_ids = std::move(tag_ids),
      .rejected_tag_ids = std::move(rejected_tag_ids),
      .pose =
          utils::ConvertOpencvTransformationMatrixToWpilibPose(feild_to_robot),
      .variance = Variance(tag_ids.size(), avg_distance, kvariance_min_,
                           kvariance_scalar_),
      .timestamp = detections[0].timestamp,
      .num_tags = num_tags,
      .avg_tag_dist = avg_distance}};
}

}  // namespace localization
