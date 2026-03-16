#include "src/gamepiece/gamepiece.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/utils/camera_utils.h"
#include "src/yolo/model_constants.h"
#include "src/yolo/yolo.h"

auto main() -> int {
  auto camera_constant =
      camera::SelectCameraConfig(camera::GetCameraConstants());
  camera::CameraSource source = camera::SelectCamera(
      "nvidia_apriltag_test", std::nullopt, camera::GetCameraConstants());
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> coral_table =
      inst.GetTable("Orin/Gamepiece/coral");
  std::shared_ptr<nt::NetworkTable> algae_table =
      inst.GetTable("Orin/Gamepiece/algae");
  nt::StructTopic<frc::Pose2d> coral_topic =
      coral_table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose2d> algae_topic =
      algae_table->GetStructTopic<frc::Pose2d>("Pose");
  yolo::ModelInfo model_info = yolo::models[yolo::Model::COLOR];
  yolo::Yolo color_model(model_info.path, model_info.color);

  std::thread usb0_gamepiece_thread(
      gamepiece::run_gamepiece_detect, std::ref(color_model),
      std::ref(model_info.class_names), std::ref(source), std::ref(coral_topic),
      std::ref(algae_topic),
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()),
      utils::ReadExtrinsics(camera_constant.extrinsics_path.value()), true);

  usb0_gamepiece_thread.join();
}
