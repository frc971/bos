#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>

#include "wpi/DataLogReader.h"
#include "wpi/MemoryBuffer.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/utils/log.h"

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

  std::error_code ec;
  auto buffer = wpi::MemoryBuffer::GetFileAsStream(file_path, ec);
  if (ec) {
    std::cerr << "Error: Could not open file " << file_path << ": "
              << ec.message() << "\n";
    return 1;
  }

  wpi::log::DataLogReader reader(std::move(buffer));
  if (!reader.IsValid()) {
    std::cerr << "Error: Data log is invalid\n";
    return 1;
  }

  std::cout << "Data log version: 0x" << std::hex << reader.GetVersion()
            << std::dec << "\n";
  std::cout << "Processing log: " << file_path << "\n\n";

  std::map<int, wpi::log::StartRecordData> entry_map;
  int target_entry_id = -1;

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

  std::cout << "Pose3d records for '" << target_entry << "':\n";
  
  std::vector<std::pair<int64_t, std::vector<uint8_t>>> pose_records;

  buffer = wpi::MemoryBuffer::GetFileAsStream(file_path, ec);
  if (ec) {
    std::cerr << "Error: Could not reopen file\n";
    return 1;
  }
  wpi::log::DataLogReader reader2(std::move(buffer));

  for (const auto& record : reader2) {
    if (!record.IsControl() && record.GetEntry() == target_entry_id) {
      const int64_t timestamp = record.GetTimestamp();
      auto raw_data = record.GetRaw();
      pose_records.emplace_back(timestamp, 
                                std::vector<uint8_t>(raw_data.begin(), raw_data.end()));
    }
  }

  std::cout << "Total Pose3d records: " << pose_records.size() << "\n\n";

  const double rotation_threshold = 0.1;
  const double translation_threshold = 0.5;

  std::cout << "Analyzing consecutive Pose3d changes:\n";
  for (size_t i = 1; i < pose_records.size(); i++) {
    const auto& prev_record = pose_records[i - 1];
    const auto& curr_record = pose_records[i];

    int64_t curr_timestamp = curr_record.first;
    const auto& prev_data = prev_record.second;
    const auto& curr_data = curr_record.second;

    try {
      if (prev_data.size() < 56 || curr_data.size() < 56) {
        LOG(ERROR) << "Invalid Pose3d data size";
        continue;
      }

      auto read_double = [](const uint8_t* data, size_t offset) -> double {
        double value;
        std::memcpy(&value, data + offset, sizeof(double));
        return value;
      };

      double prev_px = read_double(prev_data.data(), 0);
      double prev_py = read_double(prev_data.data(), 8);
      double prev_pz = read_double(prev_data.data(), 16);
      double prev_qw = read_double(prev_data.data(), 24);
      double prev_qx = read_double(prev_data.data(), 32);
      double prev_qy = read_double(prev_data.data(), 40);
      double prev_qz = read_double(prev_data.data(), 48);
      
      frc::Pose3d prev_pose(
          frc::Translation3d(units::meter_t(prev_px), units::meter_t(prev_py), 
                             units::meter_t(prev_pz)),
          frc::Rotation3d(frc::Quaternion(prev_qw, prev_qx, prev_qy, prev_qz)));

      double curr_px = read_double(curr_data.data(), 0);
      double curr_py = read_double(curr_data.data(), 8);
      double curr_pz = read_double(curr_data.data(), 16);
      double curr_qw = read_double(curr_data.data(), 24);
      double curr_qx = read_double(curr_data.data(), 32);
      double curr_qy = read_double(curr_data.data(), 40);
      double curr_qz = read_double(curr_data.data(), 48);
      
      frc::Pose3d curr_pose(
          frc::Translation3d(units::meter_t(curr_px), units::meter_t(curr_py), 
                             units::meter_t(curr_pz)),
          frc::Rotation3d(frc::Quaternion(curr_qw, curr_qx, curr_qy, curr_qz)));

      const auto transl_delta = 
          curr_pose.Translation().Distance(prev_pose.Translation());
      
      auto prev_rot = prev_pose.Rotation();
      auto curr_rot = curr_pose.Rotation();
      double prev_yaw = prev_rot.Z().value();
      double curr_yaw = curr_rot.Z().value();
      double yaw_delta = std::abs(curr_yaw - prev_yaw);
      while (yaw_delta > M_PI) yaw_delta -= 2 * M_PI;
      double rotation_delta = std::abs(yaw_delta);

      if (transl_delta.value() > translation_threshold) {
        LOG(WARNING) << "Translation delta exceeded threshold at timestamp " 
                     << curr_timestamp << " us: " << transl_delta.value() 
                     << " m (threshold: " << translation_threshold << " m)";
      }

      if (rotation_delta > rotation_threshold) {
        LOG(WARNING) << "Rotation delta exceeded threshold at timestamp " 
                     << curr_timestamp << " us: " << rotation_delta 
                     << " rad (threshold: " << rotation_threshold << " rad)";
      }

    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to deserialize Pose3d at timestamp " 
                 << curr_timestamp << ": " << e.what();
    }
  }

  return 0;
}