#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "wpi/DataLogReader.h"
#include "wpi/MemoryBuffer.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

ABSL_FLAG(std::string, input_file, "", "Path to the wpilog file");
ABSL_FLAG(std::string, entry_name, "NT:/Orin/PoseEstimate/Left/Pose3d",
          "Name of the entry to extract");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const std::string file_path = absl::GetFlag(FLAGS_input_file);
  const std::string target_entry = absl::GetFlag(FLAGS_entry_name);

  if (file_path.empty()) {
    std::cerr << "Error: --input_file flag is required\n";
    return 1;
  }

  // Load the wpilog file into a MemoryBuffer
  std::error_code ec;
  auto buffer = wpi::MemoryBuffer::GetFileAsStream(file_path, ec);
  if (ec) {
    std::cerr << "Error: Could not open file " << file_path << ": "
              << ec.message() << "\n";
    return 1;
  }

  // Create the DataLogReader
  wpi::log::DataLogReader reader(std::move(buffer));
  if (!reader.IsValid()) {
    std::cerr << "Error: Data log is invalid\n";
    return 1;
  }

  std::cout << "Data log version: 0x" << std::hex << reader.GetVersion()
            << std::dec << "\n";
  std::cout << "Processing log: " << file_path << "\n\n";

  // Map entry IDs to their start record data
  std::map<int, wpi::log::StartRecordData> entry_map;
  int target_entry_id = -1;

  // First pass: build entry map and find target entry
  for (const auto& record : reader) {
    if (record.IsStart()) {
      wpi::log::StartRecordData start_data;
      if (record.GetStartData(&start_data)) {
        entry_map[start_data.entry] = start_data;

        if (std::string(start_data.name) == target_entry) {
          target_entry_id = start_data.entry;
          std::cout << "Found target entry: " << start_data.name << "\n";
          std::cout << "  Entry ID: " << target_entry_id << "\n";
          std::cout << "  Type: " << start_data.type << "\n\n";
        }
      }
    }
  }

  if (target_entry_id == -1) {
    std::cout << "Target entry '" << target_entry << "' not found in log\n";
    return 0;
  }

  // Second pass: extract Pose3d data for target entry
  std::cout << "Pose3d records for '" << target_entry << "':\n";
  int record_count = 0;

  // Reload the reader for the second pass
  buffer = wpi::MemoryBuffer::GetFileAsStream(file_path, ec);
  if (ec) {
    std::cerr << "Error: Could not reopen file\n";
    return 1;
  }
  wpi::log::DataLogReader reader2(std::move(buffer));

  for (const auto& record : reader2) {
    if (!record.IsControl() && record.GetEntry() == target_entry_id) {
      record_count++;
      const int64_t timestamp = record.GetTimestamp();
      auto raw_data = record.GetRaw();

      std::cout << "[" << record_count << "] Timestamp: " << timestamp
                << " us, Size: " << raw_data.size() << " bytes\n";
    }
  }

  std::cout << "\n";
  std::cout << "Total Pose3d records: " << record_count << "\n";

  return 0;
}