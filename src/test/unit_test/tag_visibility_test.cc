#include <gtest/gtest.h>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/angle.h>
#include <units/length.h>

#include "src/localization/tag_visibility.h"

namespace {

auto TestCamera() -> localization::VisibilityCameraModel {
  return {.name = "test",
          .image_width = 640,
          .image_height = 480,
          .fx = 320.0,
          .fy = 320.0,
          .cx = 320.0,
          .cy = 240.0,
          .robot_to_camera = frc::Transform3d{},
          .minimum_tag_area_pixels = 0.0,
          .maximum_tag_distance_meters = 10.0};
}

auto TagAt(double x, double y, double z, double yaw_radians) -> frc::AprilTag {
  return {.ID = 1,
          .pose = frc::Pose3d{
              units::meter_t{x}, units::meter_t{y}, units::meter_t{z},
              frc::Rotation3d{units::radian_t{0.0}, units::radian_t{0.0},
                              units::radian_t{yaw_radians}}}};
}

TEST(TagVisibilityTest, SeesFrontFacingTagInFrame) {  // NOLINT
  const auto tags = std::vector<frc::AprilTag>{TagAt(2.0, 0.0, 0.0, M_PI)};
  const auto visible =
      localization::GetVisibleTags(frc::Pose3d{}, TestCamera(), tags);

  ASSERT_EQ(visible.size(), 1U);
  EXPECT_EQ(visible.front().id, 1);
  EXPECT_NEAR(visible.front().image_corners[0].x, 333.208, 0.01);
  EXPECT_NEAR(visible.front().image_corners[0].y, 226.792, 0.01);
}

TEST(TagVisibilityTest, RejectsBackFacingAndOutOfFrameTags) {  // NOLINT
  auto camera = TestCamera();
  EXPECT_TRUE(localization::GetVisibleTags(frc::Pose3d{}, camera,
                                           {TagAt(2.0, 0.0, 0.0, 0.0)})
                  .empty());
  EXPECT_TRUE(localization::GetVisibleTags(frc::Pose3d{}, camera,
                                           {TagAt(2.0, 3.0, 0.0, M_PI)})
                  .empty());
}

TEST(TagVisibilityTest, AppliesAreaAndDistanceThresholds) {  // NOLINT
  auto camera = TestCamera();
  const auto tag = TagAt(2.0, 0.0, 0.0, M_PI);

  camera.maximum_tag_distance_meters = 1.0;
  EXPECT_TRUE(
      localization::GetVisibleTags(frc::Pose3d{}, camera, {tag}).empty());

  camera.maximum_tag_distance_meters = 10.0;
  camera.minimum_tag_area_pixels = 1000.0;
  EXPECT_TRUE(
      localization::GetVisibleTags(frc::Pose3d{}, camera, {tag}).empty());
}

}  // namespace
