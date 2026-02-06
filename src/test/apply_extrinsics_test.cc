#include <frc/geometry/Pose3d.h>
#include <opencv2/opencv.hpp>
#include "src/localization/position_solver.h"

const int ktag_id = 26;

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

auto main() -> int {
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 100, 0.0, 0.0);
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);

  const double translation_x = tvec.ptr<double>()[2];
  const double translation_y = tvec.ptr<double>()[0];
  const double translation_z = tvec.ptr<double>()[1];

  const double rotation_x = rvec.ptr<double>()[2];
  const double rotation_y = rvec.ptr<double>()[0];
  const double rotation_z = rvec.ptr<double>()[1];

  {
    auto pose = frc::Pose3d(frc::Translation3d(units::meter_t{translation_x},
                                               units::meter_t{translation_y},
                                               units::meter_t{translation_z}),
                            frc::Rotation3d(units::radian_t{rotation_x},
                                            units::radian_t{rotation_y},
                                            units::radian_t{rotation_z}));

    Eigen::Vector3d rotation_vector(rotation_x, rotation_y, rotation_z);

    auto rotation = frc::Rotation3d(rotation_vector,
                                    units::radian_t{rotation_vector.norm()});

    rotation.RotateBy(frc::Rotation3d(0_deg, 0_deg, 180_deg));

    frc::Transform3d camera_to_tag(units::meter_t{pose.X()},
                                   units::meter_t{-pose.Y()},
                                   units::meter_t{-pose.Z()}, rotation);

    frc::Transform3d tag_to_camera = camera_to_tag.Inverse();

    frc::Pose3d tag_pose =
        localization::kapriltag_layout.GetTagPose(ktag_id).value();

    frc::Pose3d camera_pose = tag_pose.TransformBy(tag_to_camera);
    utils::PrintPose3d(camera_pose);
  }

  {
    auto camera_to_tag = MakeTransform(rvec, tvec);
    auto tag_to_camera = camera_to_tag.inv();

    auto feild_to_tag = EigenToCvMat(
        localization::kapriltag_layout.GetTagPose(ktag_id).value().ToMatrix());

    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, std::numbers::pi);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    auto rotate_z = MakeTransform(rvec, tvec);
    LOG(INFO) << "\n" << feild_to_tag;
    LOG(INFO) << "\n" << feild_to_tag * rotate_z;
    LOG(INFO) << "\n" << feild_to_tag * rotate_z * tag_to_camera;
    cv::Mat a = feild_to_tag * rotate_z * tag_to_camera;
    LOG(INFO) << a;
    Eigen::Matrix4d b;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        b(i, j) = a.at<double>(i, j);
        std::cout << a.at<double>(i, j) << " ";
      }
      std::cout << "\n";
    }
    LOG(INFO) << b;
    frc::Pose3d c(b);
  }
}
