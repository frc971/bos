#pragma once

#include <cstdint>

namespace utils {

void InitializeWpiLogTime();
auto GetLogTimestampSeconds() -> double;
auto GetLogTimestampMicros() -> int64_t;
auto GetNonzeroLogTimestampMicros() -> int64_t;
auto IsTimestampSane(double timestamp_seconds) -> bool;
auto WaitForSaneTimestamp() -> bool;

}  // namespace utils
