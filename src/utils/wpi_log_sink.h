#pragma once
#include <frc/DataLogManager.h>
#include <networktables/StringTopic.h>
#include <wpi/DataLog.h>
#include "absl/log/log.h"
#include "absl/log/log_sink.h"

namespace utils {
class WpiLogSink : public absl::LogSink {
 public:
  WpiLogSink();
  void Send(const absl::LogEntry& entry) override;

 private:
  wpi::log::StringLogEntry string_log_entry_;
};
}  // namespace utils
