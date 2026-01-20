#include <frc/DataLogManager.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <wpi/DataLog.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <vector>
#include "src/pathing/pathing.h"

frc::Field2d field;

void processPose(const frc::Pose2d& pose,
                 nt::StructPublisher<frc::Pose2d>& poseLog) {
  field.SetRobotPose(pose);
  poseLog.Set(pose);
  field.GetObject("Robot")->SetPose(pose);
}

auto main() -> int {
  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartServer();

  frc::DataLogManager::Start("/root/bos/logs");
  frc::DataLogManager::LogNetworkTables(true);

  frc::SmartDashboard::PutData("Field", &field);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    std::cerr << "Failed to open navgrid.json" << std::endl;
    return 1;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_W = data["grid"][0].size();
  const int GRID_H = data["grid"].size();
  double nodeSizeMeters = data["nodeSizeMeters"];

  std::vector<std::vector<bool>> gridData(GRID_H, std::vector<bool>(GRID_W));

  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      gridData[y][x] = data["grid"][y][x];
    }
  }

  cv::Mat grid = initializeGrid(gridData);

  int start_x = 10;
  int start_y = 5;
  int target_x = 45;
  int target_y = 22;

  std::vector<frc::Pose2d> poses =
      createSpline(grid, start_x, start_y, target_x, target_y, nodeSizeMeters);

  if (poses.empty()) {
    std::cout << "No path found!\n";
    return 1;
  }

  auto poseTopic = inst.GetStructTopic<frc::Pose2d>("/replayPose");
  auto poseLog = poseTopic.Publish();

  for (const auto& pose : poses) {
    processPose(pose, poseLog);
    inst.Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  frc::DataLogManager::Stop();

  return 0;
}