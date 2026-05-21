#include "src/utils/log_path.h"

#include <string>

#include "absl/flags/flag.h"

ABSL_FLAG(std::string, sim_log_name, "sim.wpilog",  // NOLINT
          "Name of the simulation WPILib log file.");

namespace utils {

std::string GetSimLogName() {
  return absl::GetFlag(FLAGS_sim_log_name);
}

}  // namespace utils
