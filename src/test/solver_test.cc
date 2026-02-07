#include <gtest/gtest.h>
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"

namespace frc {
void PrintTo(const frc::Pose3d& pose, std::ostream* os) {
  *os << "Pose{"
      << "t = [" << pose.Translation().X().value() << ", "
      << pose.Translation().Y().value() << ", "
      << pose.Translation().Z().value() << "], "
      << "R = [" << pose.Rotation().X().value() << ", "
      << pose.Rotation().Y().value() << ", " << pose.Rotation().Z().value()
      << "]"
      << "}";
}
}  // namespace frc

constexpr int kimage_tag_width = 20;
constexpr int kimage_tag_height = 20;
constexpr std::array<int, 2> ktag_ids = {15, 31};
constexpr int kimage_width = 20;
constexpr int kimage_height = 20;
const std::array<cv::Point2f, 4> image_points = {
    cv::Point2f(kimage_width / 2.0 - kimage_tag_width / 2.0,
                kimage_height / 2.0 + kimage_tag_height / 2.0),
    cv::Point2f(kimage_width / 2.0 + kimage_tag_width / 2.0,
                kimage_height / 2.0 + kimage_tag_height / 2.0),
    cv::Point2f(kimage_width / 2.0 + kimage_tag_width / 2.0,
                kimage_height / 2.0 - kimage_tag_height / 2.0),
    cv::Point2f(kimage_width / 2.0 - kimage_tag_width / 2.0,
                kimage_height / 2.0 - kimage_tag_height / 2.0)};

TEST(SolverTest, Basic) {
  localization::SquareSolver solver(camera::Camera::DUMMY_CAMERA);

  for (const int id : ktag_ids) {
    const localization::tag_detection_t fake_detection{.tag_id = id,
                                                       .corners = image_points};
    const std::vector<localization::tag_detection_t> fake_detections{
        fake_detection};

    localization::position_estimate_t estimate =
        solver.EstimatePosition(fake_detections)[0];
    std::cout << ::testing::PrintToString(estimate.pose) << std::endl;
    frc::Pose3d flipped_tag =
        localization::kapriltag_layout.GetTagPose(id).value();
    flipped_tag = frc::Pose3d(
        flipped_tag.Translation(),
        frc::Rotation3d(
            flipped_tag.Rotation().X(), flipped_tag.Rotation().Y(),
            flipped_tag.Rotation().Z() + units::radian_t{-std::numbers::pi}));
    EXPECT_EQ(estimate.pose, flipped_tag);
  }
}
