#include "nt_utils.h"
#include <frc/DataLogManager.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <iostream>
#include "src/utils/log.h"
namespace utils {

void StartNetworktables(int team_number) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(team_number);
  frc::DataLogManager::Start("/bos/logs/");
  LOG(INFO) << "Started networktables!" << std::endl;
}
}  // namespace utils
