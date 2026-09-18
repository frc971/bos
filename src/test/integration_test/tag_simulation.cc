#include <chrono>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <units/angle.h>
#include <units/length.h>

#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/tag_visibility.h"

ABSL_FLAG(std::string, tag_sim_camera_constants,
          "/bos/constants/camera_constants.json", "Camera constants JSON path");
ABSL_FLAG(std::string, tag_sim_cameras, "main_bot_left,main_bot_right",
          "Comma-separated camera names to simulate");
ABSL_FLAG(bool, tag_sim_follow_drive_pose, true,
          "Read the robot pose from /DriveState/Pose");
ABSL_FLAG(double, tag_sim_x, 1.0, "Fixed robot x in meters");
ABSL_FLAG(double, tag_sim_y, 1.0, "Fixed robot y in meters");
ABSL_FLAG(double, tag_sim_heading_degrees, 0.0,
          "Fixed robot heading in degrees");
ABSL_FLAG(int, tag_sim_team, 971, "NetworkTables team number");

namespace {

class CameraTopics {
 public:
  CameraTopics(nt::NetworkTableInstance instance,
               localization::VisibilityCameraModel camera)
      : camera(std::move(camera)) {
    auto table = instance.GetTable("TagSimulation/" + this->camera.name);
    visible_poses =
        table->GetStructArrayTopic<frc::Pose2d>("VisibleTags").Publish();
    hidden_poses =
        table->GetStructArrayTopic<frc::Pose2d>("HiddenTags").Publish();
    visible_ids = table->GetIntegerArrayTopic("VisibleTagIds").Publish();
    expected_count = table->GetIntegerTopic("ExpectedTagCount").Publish();
    camera_pose = table->GetStructTopic<frc::Pose3d>("CameraPose").Publish();
  }

  void Publish(const frc::Pose3d& robot_pose,
               const std::vector<frc::AprilTag>& tags) {
    const std::vector<localization::VisibleTag> visible =
        localization::GetVisibleTags(robot_pose, camera, tags);
    std::set<int> visible_id_set;
    std::vector<int64_t> ids;
    std::vector<frc::Pose2d> visible_tag_poses;
    ids.reserve(visible.size());
    visible_tag_poses.reserve(visible.size());
    for (const localization::VisibleTag& tag : visible) {
      visible_id_set.insert(tag.id);
      ids.push_back(tag.id);
      visible_tag_poses.push_back(tag.pose.ToPose2d());
    }

    std::vector<frc::Pose2d> hidden_tag_poses;
    hidden_tag_poses.reserve(tags.size() - visible.size());
    for (const frc::AprilTag& tag : tags) {
      if (!visible_id_set.contains(tag.ID)) {
        hidden_tag_poses.push_back(tag.pose.ToPose2d());
      }
    }

    visible_poses.Set(visible_tag_poses);
    hidden_poses.Set(hidden_tag_poses);
    visible_ids.Set(ids);
    expected_count.Set(static_cast<int64_t>(visible.size()));
    camera_pose.Set(robot_pose.TransformBy(camera.robot_to_camera));
  }

  localization::VisibilityCameraModel camera;
  nt::StructArrayPublisher<frc::Pose2d> visible_poses;
  nt::StructArrayPublisher<frc::Pose2d> hidden_poses;
  nt::IntegerArrayPublisher visible_ids;
  nt::IntegerPublisher expected_count;
  nt::StructPublisher<frc::Pose3d> camera_pose;
};

auto SplitCameraNames(const std::string& camera_names)
    -> std::vector<std::string> {
  std::vector<std::string> result;
  std::stringstream stream(camera_names);
  for (std::string name; std::getline(stream, name, ',');) {
    if (!name.empty()) {
      result.push_back(name);
    }
  }
  return result;
}

auto Pose3dFromPose2d(const frc::Pose2d& pose) -> frc::Pose3d {
  return frc::Pose3d{pose.X(), pose.Y(), units::meter_t{0.0},
                     frc::Rotation3d{units::radian_t{0.0}, units::radian_t{0.0},
                                     pose.Rotation().Radians()}};
}

}  // namespace

auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);

  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.StartClient4("tag_simulation");
  instance.SetServerTeam(absl::GetFlag(FLAGS_tag_sim_team));

  const camera::camera_constants_t camera_constants =
      camera::GetCameraConstants(absl::GetFlag(FLAGS_tag_sim_camera_constants));
  std::vector<std::unique_ptr<CameraTopics>> camera_topics;
  for (const std::string& name :
       SplitCameraNames(absl::GetFlag(FLAGS_tag_sim_cameras))) {
    camera_topics.push_back(std::make_unique<CameraTopics>(
        instance,
        localization::MakeVisibilityCameraModel(camera_constants.at(name))));
  }

  const auto drive_pose =
      instance.GetStructTopic<frc::Pose2d>("/DriveState/Pose")
          .Subscribe(frc::Pose2d{});
  auto robot_pose_publisher =
      instance.GetStructTopic<frc::Pose2d>("/TagSimulation/RobotPose")
          .Publish();
  const std::vector<frc::AprilTag>& tags =
      localization::kapriltag_layout.GetTags();

  while (true) {
    frc::Pose2d robot_pose_2d{
        units::meter_t{absl::GetFlag(FLAGS_tag_sim_x)},
        units::meter_t{absl::GetFlag(FLAGS_tag_sim_y)},
        units::degree_t{absl::GetFlag(FLAGS_tag_sim_heading_degrees)}};
    if (absl::GetFlag(FLAGS_tag_sim_follow_drive_pose)) {
      robot_pose_2d = drive_pose.Get();
    }
    const frc::Pose3d robot_pose = Pose3dFromPose2d(robot_pose_2d);

    robot_pose_publisher.Set(robot_pose_2d);
    for (const auto& topics : camera_topics) {
      topics->Publish(robot_pose, tags);
    }
    instance.Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
