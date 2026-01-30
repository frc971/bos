#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>
#include <opencv2/calib3d.hpp>
#include "src/camera/camera_constants.h"
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

// https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf
// Big tag size = close to tag
// Small tag size = far away from tag
constexpr double ktag_size = 0.1651;  // meters
const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout("/bos/constants/2026-rebuilt-andymark.json");
constexpr int kimage_tag_size = 50;
constexpr int ktag_id = 31;

auto createTransformMatrix(double rx, double ry, double rz, double tx,
                           double ty, double tz) -> cv::Mat {
  // --- Rotation matrices ---
  cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(rx), -sin(rx), 0,
                sin(rx), cos(rx));

  cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(ry), 0, sin(ry), 0, 1, 0,
                -sin(ry), 0, cos(ry));

  cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(rz), -sin(rz), 0, sin(rz),
                cos(rz), 0, 0, 0, 1);

  // Combined rotation: Z * Y * X
  cv::Mat R = Rz * Ry * Rx;

  // --- Translation vector ---
  cv::Mat t = (cv::Mat_<double>(3, 1) << tx, ty, tz);

  // --- 4x4 Homogeneous transformation matrix ---
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);  // initialize identity
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));       // top-left 3x3 rotation
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));       // top-right 3x1 translation

  return T;
}

auto createTransformMatrix(frc::Pose3d transform) -> cv::Mat {
  return createTransformMatrix(
      transform.Rotation().Y().to<double>(),
      transform.Rotation().Z().to<double>(),
      transform.Rotation().X().to<double>(), transform.Y().to<double>(),
      transform.Z().to<double>(), transform.X().to<double>());
}

auto createTransformMatrix(frc::Transform3d transform) -> cv::Mat {
  return createTransformMatrix(
      transform.Rotation().Y().to<double>(),
      transform.Rotation().Z().to<double>(),
      transform.Rotation().X().to<double>(), transform.Y().to<double>(),
      transform.Z().to<double>(), transform.X().to<double>());
}

auto createTransformMatrix(cv::Mat rvec, cv::Mat tvec) -> cv::Mat {
  return createTransformMatrix(rvec.at<double>(0), rvec.at<double>(1),
                               rvec.at<double>(2), tvec.at<double>(0),
                               tvec.at<double>(1), tvec.at<double>(2));
}

auto main() -> int {
  std::vector<cv::Point3d> apriltag_corners = {
      {-ktag_size / 2, ktag_size / 2, 0},
      {ktag_size / 2, ktag_size / 2, 0},
      {ktag_size / 2, -ktag_size / 2, 0},
      {-ktag_size / 2, -ktag_size / 2, 0}};

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // output rotation vector
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // output translation vector
  auto config = camera::Camera::STOVETOP_BOT_FRONT_RIGHT;
  auto camera_matrix = camera_matrix_from_json<cv::Mat>(
      utils::read_intrinsics(camera::camera_constants[config].intrinsics_path));

  auto distortion_coefficients = distortion_coefficients_from_json<cv::Mat>(
      utils::read_intrinsics(camera::camera_constants[config].intrinsics_path));

  std::vector<cv::Point2d> image_points = {
      cv::Point2f(1000 - kimage_tag_size, 500 + kimage_tag_size),
      cv::Point2f(1000 + kimage_tag_size, 500 + kimage_tag_size),
      cv::Point2f(1000 + kimage_tag_size, 500 - kimage_tag_size),
      cv::Point2f(1000 - kimage_tag_size, 500 - kimage_tag_size),
  };

  auto tag_pose = kapriltag_layout.GetTagPose(ktag_id).value();
  auto feild_to_tag = createTransformMatrix(tag_pose);

  for (auto& apriltag_corner : apriltag_corners) {
    apriltag_corner.x = -apriltag_corner.x;
    apriltag_corner.y = -apriltag_corner.y;

    cv::Mat pt = (cv::Mat_<double>(4, 1) << apriltag_corner.x,
                  apriltag_corner.y, apriltag_corner.z, 1.0);
    cv::Mat result = feild_to_tag * pt;
    apriltag_corner = cv::Point3d(result.at<double>(0), result.at<double>(1),
                                  result.at<double>(2));
  }

  // [886.039, 530.664][1088.88, 521.248][1074.46, 333.045][880.557, 333.923]
  cv::solvePnP(apriltag_corners, image_points, camera_matrix,
               distortion_coefficients, rvec, tvec, false,
               cv::SOLVEPNP_ITERATIVE);

  LOG(INFO) << "\n" << rvec;
  LOG(INFO) << "\n" << tvec;

  std::vector<cv::Point2d> projectedPoints;
  cv::projectPoints(apriltag_corners, rvec, tvec, camera_matrix,
                    distortion_coefficients, projectedPoints);

  // Compute reprojection error
  double totalError = 0.0;
  for (size_t i = 0; i < image_points.size(); i++) {
    double err = cv::norm(image_points[i] - projectedPoints[i]);
    totalError += err * err;
  }

  double meanError = std::sqrt(totalError / image_points.size());
  std::cout << "Mean reprojection error: " << meanError << " pixels"
            << std::endl;

  std::cout << "feild_to_camera\n";
  std::cout << createTransformMatrix(rvec, tvec);

  // auto camera_to_tag = createTransformMatrix(rvec, tvec);
  // auto tag_to_camera = camera_to_tag.inv();
  //
  // auto tag_pose = kapriltag_layout.GetTagPose(ktag_id).value();
  // auto feild_to_tag = createTransformMatrix(tag_pose);
  // auto tag_to_feild = feild_to_tag.inv();
  //
  // std::cout << "feild to tag\n";
  // std::cout << feild_to_tag << std::endl;
  //
  // auto feild_to_camera = feild_to_tag * camera_to_tag.inv();
  //
  // std::cout << "feild to camera\n";
  // std::cout << feild_to_camera << std::endl;

  return 0;
}
