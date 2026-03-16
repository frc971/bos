#include "src/utils/wpi_log_sink.h"

namespace utils {

WpiLogSink::WpiLogSink()
    : string_log_entry_(frc::DataLogManager::GetLog(), "BOS") {}

void WpiLogSink::Send(const absl::LogEntry& entry) {
  string_log_entry_.Append(entry.encoded_message());
}

}  // namespace utils
