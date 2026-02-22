#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

constexpr auto rad(double deg) -> double {
  return deg / 180 * M_PI;
}

auto TransformationMatrix(const double rx, const double ry, const double rz,
                          const double x, const double y, const double z)
    -> Eigen::Matrix4d {
  std::cout << "rx: " << rx << "rx: " << ry << "rx: " << rz << std::endl;
  Eigen::Matrix3d Rx, Ry, Rz;

  Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);

  Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);

  Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;

  Eigen::Matrix3d R = Rz * Ry * Rx;
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
  A.block<3, 3>(0, 0) = R;
  A(0, 3) = x;
  A(1, 3) = y;
  A(2, 3) = z;

  return A;
}

TEST(JointSolveTest, EstimatePosition) {  // NOLINT
  const Eigen::Matrix4d transform =
      TransformationMatrix(rad(35), rad(23), rad(58), 1, 2, 3);
  const utils::TransformValues decomp =
      utils::ExtractTranslationAndRotation(transform);
  std::cout << decomp.x << " " << decomp.y << " " << decomp.z << " "
            << decomp.rx << " " << decomp.ry << " " << decomp.rz << " "
            << std::endl;
  const std::array<Eigen::Matrix4d, 4> decomped =
      utils::SeparateTranslationAndRotationMatrices(decomp);
  utils::PrintTransformationMatrix(utils::EigenToCvMat(transform), "original");
  utils::PrintTransformationMatrix(
      utils::EigenToCvMat(decomped[0] * decomped[1] * decomped[2] *
                          decomped[3]),
      "new");
}
