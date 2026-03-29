#include "src/utils/wpilog_time.h"

#include <chrono>
#include <thread>

#include <frc/RobotController.h>
#include <wpi/timestamp.h>

#include "src/utils/log.h"

namespace {

constexpr double kMaxSaneRobotTimestampSeconds = 1'000'000.0;
constexpr auto kTimestampWaitTimeout = std::chrono::seconds(5);
constexpr auto kTimestampPollInterval = std::chrono::milliseconds(20);

}  // namespace

namespace utils {

namespace {

auto GetFPGATimeMicrosNow() -> uint64_t {
  return frc::RobotController::GetFPGATime();
}

}  // namespace

void InitializeWpiLogTime() {
  wpi::SetNowImpl(GetFPGATimeMicrosNow);
}

auto GetLogTimestampSeconds() -> double {
  return static_cast<double>(GetLogTimestampMicros()) / 1e6;
}

auto GetLogTimestampMicros() -> int64_t {
  return static_cast<int64_t>(frc::RobotController::GetFPGATime());
}

auto GetNonzeroLogTimestampMicros() -> int64_t {
  return std::max<int64_t>(1, GetLogTimestampMicros());
}

auto IsTimestampSane(double timestamp_seconds) -> bool {
  return timestamp_seconds >= 0.0 &&
         timestamp_seconds < kMaxSaneRobotTimestampSeconds;
}

auto WaitForSaneTimestamp() -> bool {
  const auto deadline = std::chrono::steady_clock::now() + kTimestampWaitTimeout;
  double timestamp = GetLogTimestampSeconds();
  while (!IsTimestampSane(timestamp)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      LOG(WARNING) << "Timed out waiting for sane FPGA timestamp. Continuing with "
                   << timestamp;
      return false;
    }
    std::this_thread::sleep_for(kTimestampPollInterval);
    timestamp = GetLogTimestampSeconds();
  }
  return true;
}

}  // namespace utils
