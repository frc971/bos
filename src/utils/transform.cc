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

template <typename Derived>
auto EigenToCvMat(const Eigen::MatrixBase<Derived>& mat) -> cv::Mat {
  cv::Mat cvMat(mat.rows(), mat.cols(), CV_64F);
  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      cvMat.ptr<double>(), mat.rows(), mat.cols()) = mat;
  return cvMat;
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

auto OpencvCoordinateToWpilib(cv::Mat& vec) -> void {
  const double x = vec.ptr<double>()[2];
  const double y = vec.ptr<double>()[0];
  const double z = vec.ptr<double>()[1];
  vec.ptr<double>()[0] = x;
  vec.ptr<double>()[1] = -y;
  vec.ptr<double>()[2] = -z;
}

auto OpencvTransformationMatrixToPose3d(const cv::Mat& transform)
    -> frc::Pose3d {
  cv::Mat R = transform(cv::Range(0, 3), cv::Range(0, 3)).clone();
  cv::Mat tvec = transform(cv::Range(0, 3), cv::Range(3, 4)).clone();
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  OpencvCoordinateToWpilib(tvec);
  OpencvCoordinateToWpilib(rvec);
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

auto Point3dToHomogenizedMat(cv::Point3d point) -> cv::Mat {
  cv::Mat mat = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z, 1);
  return mat;
}

auto CvMatToPoint3f(cv::Mat mat) -> cv::Point3d {
  return {mat.at<double>(0), mat.at<double>(1), mat.at<double>(2)};
}

auto HomogenizePoint3d(cv::Point3d point) -> cv::Mat {
  return (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);  // NOLINT
}

template cv::Mat utils::EigenToCvMat<Eigen::Matrix<double, 4, 4>>(
    const Eigen::MatrixBase<Eigen::Matrix<double, 4, 4>>&);
}  // namespace utils
