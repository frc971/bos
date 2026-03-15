#pragma once
#include "src/utils/pch.h"
namespace utils {
// Sets up logging and networktables. Should be called at the beggining of every robot's main
void StartNetworktables(int team_number = 971);
auto GetNewLogPath(const std::string& log_dir = "/bos/logs") -> std::string;
}  // namespace utils
