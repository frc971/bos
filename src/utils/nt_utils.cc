#include "nt_utils.h"
#include <frc/DataLogManager.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <iostream>
namespace utils {

void StartNetworktables(int team_number) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(team_number);
  frc::DataLogManager::Start("/bos/logs/");
  std::cout << "Started networktables!" << std::endl;
}
}  // namespace utils
