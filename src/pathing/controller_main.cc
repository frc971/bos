#include "src/pathing/controller.h"
#include <networktables/NetworkTableInstance.h>
#include <chrono>
#include <thread>
#include <iostream>

int main() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  auto table = instance.GetTable("Pathing");
  
  auto enabled_entry = table->GetEntry("Enabled");

  pathing::Controller* controller = nullptr;
  std::thread worker;
  bool last = false;

  while (true) {
    bool enabled = enabled_entry.GetBoolean(false);
    if (enabled && !last) {
      controller = new pathing::Controller(); 
      worker = std::thread([controller]() { controller->Send(); });
    } else if (!enabled && last) {
      if (controller) {
        controller->Stop();
        if (worker.joinable()) worker.join();
        delete controller;
        controller = nullptr;
      }
    }
    last = enabled;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
