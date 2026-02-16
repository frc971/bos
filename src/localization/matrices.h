#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace localization {

const cv::Mat cv_to_wpilib = (cv::Mat_<double>(4, 4) << 0, 0, 1, 0, -1, 0, 0, 0,
                              0, -1, 0, 0, 0, 0, 0, 1);
const cv::Mat wpilib_to_cv = cv_to_wpilib.t();

auto inline ChangeBasis(const cv::Mat& mat, const cv::Mat& basis) {
  return basis * mat *
         basis
             .t();  // TODO should be inverse, but it works with transpose. Need to check if this is valid for all basis changes we might want to do
}

auto TransformationMatrix(const double rx, const double ry, const double rz,
                          const double x, const double y, const double z)
    -> Eigen::Matrix4d {
  Eigen::Matrix3d roll_mat;
  roll_mat << std::cos(rz), -std::sin(rz), 0.0, std::sin(rz), std::cos(rz), 0.0,
      0.0, 0.0, 1.0;

  Eigen::Matrix3d pitch_mat;
  pitch_mat << std::cos(rx), 0.0, std::sin(rx), 0.0, 1.0, 0.0, -std::sin(rx),
      0.0, std::cos(rx);

  Eigen::Matrix3d yaw_mat;
  yaw_mat << 1.0, 0.0, 0.0, 0.0, std::cos(ry), -std::sin(ry), 0.0, std::sin(ry),
      std::cos(ry);

  const Eigen::Matrix3d R = roll_mat * pitch_mat * yaw_mat;

  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
  A.block<3, 3>(0, 0) = R;
  A(0, 3) = x;
  A(1, 3) = y;
  A(2, 3) = z;

  return A;
}

}  // namespace localization
