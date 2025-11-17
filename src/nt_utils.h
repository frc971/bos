#pragma once

#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>

class NTUtils {
 public:
  static bool started;

  NTUtils() = delete;

  static bool start_networktables();
};
