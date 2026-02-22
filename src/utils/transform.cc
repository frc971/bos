#include "src/utils/transform.h"

namespace utils {

auto MakeTransform(const cv::Mat& rvec, const cv::Mat& tvec) -> cv::Mat {
  CV_Assert(rvec.total() == 3 && tvec.total() == 3);

  cv::Mat R;
  cv::Rodrigues(rvec, R);  // 3x3

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

  R.copyTo(T(cv::Rect(0, 0, 3, 3)));

  T.at<double>(0, 3) = tvec.at<double>(0);
  T.at<double>(1, 3) = tvec.at<double>(1);
  T.at<double>(2, 3) = tvec.at<double>(2);

  return T;
}

auto CvMatToEigen(const cv::Mat& mat) -> Eigen::Matrix4d {
  Eigen::Matrix4d out;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      out(i, j) = mat.at<double>(i, j);
    }
  }
  return out;
}

auto ConvertOpencvCoordinateToWpilib(cv::Mat& vec) -> void {
  const double x = vec.ptr<double>()[2];
  const double y = vec.ptr<double>()[0];
  const double z = vec.ptr<double>()[1];
  vec.ptr<double>()[0] = x;
  vec.ptr<double>()[1] = -y;
  vec.ptr<double>()[2] = -z;
}

auto ConvertOpencvTransformationMatrixToWpilibPose(const cv::Mat& transform)
    -> frc::Pose3d {
  cv::Mat R = transform(cv::Range(0, 3), cv::Range(0, 3)).clone();
  cv::Mat tvec = transform(cv::Range(0, 3), cv::Range(3, 4)).clone();
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  ConvertOpencvCoordinateToWpilib(tvec);
  ConvertOpencvCoordinateToWpilib(rvec);
  cv::Mat wpilib_transform = MakeTransform(rvec, tvec);
  return frc::Pose3d(CvMatToEigen(wpilib_transform));
}

auto Pose3dToCvMat(frc::Pose3d pose) -> cv::Mat {
  frc::Pose3d opencv_pose(
      frc::Translation3d(-units::meter_t{pose.Y().value()},
                         -units::meter_t{pose.Z().value()},
                         units::meter_t{pose.X().value()}),
      frc::Rotation3d(-units::radian_t{pose.Rotation().Y()},
                      -units::radian_t{pose.Rotation().Z()},
                      units::radian_t{pose.Rotation().X()}));
  return utils::EigenToCvMat(opencv_pose.ToMatrix());
}

auto HomogenizePoint3d(cv::Point3d point) -> cv::Mat {
  return (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);  // NOLINT
}

template cv::Mat utils::EigenToCvMat<Eigen::Matrix<double, 4, 4>>(
    const Eigen::MatrixBase<Eigen::Matrix<double, 4, 4>>&);

auto ExtractTranslationAndRotation(const Eigen::Matrix4d& transform_mat)
    -> TransformValues {
  double x = transform_mat(0, 3);
  double y = transform_mat(1, 3);
  double z = transform_mat(2, 3);

  const Eigen::Matrix3d& R = transform_mat.block<3, 3>(0, 0);

  double sy = std::hypot(R(0, 0), R(1, 0));

  bool singular = sy < 1e-6;

  double roll, pitch, yaw;

  if (!singular) {
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    // Gimbal lock
    roll = std::atan2(-R(1, 2), R(1, 1));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = 0.0;
  }

  return {x, y, z, roll, pitch, yaw};
}

auto SeparateTranslationAndRotationMatrices(
    const TransformValues& decomposition) -> std::array<Eigen::Matrix4d, 4> {
  std::array<Eigen::Matrix4d, 4> output;

  for (int i = 0; i < output.size(); i++) {
    output[i] = Eigen::Matrix4d::Identity();
  }

  // clang-format off
  const Eigen::Matrix3d Rx = (Eigen::Matrix3d() << 
      1, 0, 0,
      0, cos(decomposition.rx), -sin(decomposition.rx),
      0, sin(decomposition.rx), cos(decomposition.rx)).finished();
  const Eigen::Matrix3d Ry = (Eigen::Matrix3d() << 
      cos(decomposition.ry), 0, sin(decomposition.ry),
      0, 1, 0,
      -sin(decomposition.ry), 0, cos(decomposition.ry)).finished();
  const Eigen::Matrix3d Rz = (Eigen::Matrix3d() << 
      cos(decomposition.rz), -sin(decomposition.rz), 0,
      sin(decomposition.rz), cos(decomposition.rz), 0,
      0, 0, 1).finished();
  // clang-format on
  output[0](0, 3) = decomposition.x;
  output[0](1, 3) = decomposition.y;
  output[0](2, 3) = decomposition.z;
  output[1].block<3, 3>(0, 0) = Rz;
  output[2].block<3, 3>(0, 0) = Ry;
  output[3].block<3, 3>(0, 0) = Rx;

  return output;
}
}  // namespace utils
