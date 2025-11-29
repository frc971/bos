#include "nt_utils.h"
#include <iostream>

bool NTUtils::started = false;

bool NTUtils::start_networktables() {
  if (started) {
    return false;
  }
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  frc::DataLogManager::Start("/bos/logs/");
  std::cout << "Started networktables!" << std::endl;
  started = true;
  return true;
}
