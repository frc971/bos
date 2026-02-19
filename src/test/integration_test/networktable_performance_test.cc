#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include "src/utils/log.h"
#include "src/utils/nt_utils.h"
#include "src/utils/pch.h"

auto main() -> int {
  utils::StartNetworktables();

  auto instance = nt::NetworkTableInstance::GetDefault();

  std::shared_ptr<nt::NetworkTable> table = instance.GetTable("DriveState");

  auto pose_subscriber =
      table->GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d());

  using clock = std::chrono::high_resolution_clock;
  auto last = clock::now();
  int iterations = 0;

  for (;;) {
    while (pose_subscriber.ReadQueue().empty()) {}

    iterations++;
    auto now = clock::now();
    double elapsed_ms =
        std::chrono::duration<double, std::milli>(now - last).count();

    if (elapsed_ms >= 1000.0) {
      std::cout << "Frequency: " << iterations / (elapsed_ms / 1000.0)
                << " Hz\n";
      std::cout << "Period: " << elapsed_ms / iterations << " ms\n";
      iterations = 0;
      last = now;
    }
  }
}
