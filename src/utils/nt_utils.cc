#include "nt_utils.h"
#include <frc/DataLogManager.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "src/utils/log.h"
namespace utils {

void StartNetworktables(int team_number) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopServer();
  inst.StopClient();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(team_number);
  inst.StartDSClient();

  LOG(INFO) << "Team number" << team_number;
  LOG(INFO) << "Waiting for connection";
  while (!inst.IsConnected()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  LOG(INFO) << "Connected to rio!";
}
}  // namespace utils
