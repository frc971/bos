#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <variant>
#include <vector>
#include <cmath>

#include "wpi/DataLogReader.h"
#include "wpi/json.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

using NTValue = std::variant<double, float, int64_t, bool, std::string,
                             std::vector<double>, std::vector<float>,
                             std::vector<int64_t>, frc::Pose3d>;


bool decode_record(const wpi::log::DataLogRecord& record,
                   const wpi::log::StartRecordData& start_data,
                   NTValue& out_value) {

    if (auto d = record.GetValue<double>()) {
        out_value = *d;
    } else if (auto f = record.GetValue<float>()) {
        out_value = *f;
    } else if (auto i = record.GetValue<int64_t>()) {
        out_value = *i;
    } else if (auto b = record.GetValue<bool>()) {
        out_value = *b;
    } else if (auto s = record.GetValue<std::string>()) {
        out_value = *s;
    } else if (auto arr_d = record.GetValue<std::vector<double>>()) {
        out_value = *arr_d;
    } else if (auto arr_f = record.GetValue<std::vector<float>>()) {
        out_value = *arr_f;
    } else if (auto arr_i = record.GetValue<std::vector<int64_t>>()) {
        out_value = *arr_i;
    } else {
        // fallback: treat raw as Pose3d if needed
        if (auto raw = record.GetValue<frc::Pose3d>()) {
            out_value = *raw;
        } else {
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    absl::ParseCommandLine(argc, argv);

    std::string log_path = "path_to_log.dlog";
    wpi::log::DataLogReader reader(log_path);

    std::map<std::string, std::vector<std::pair<int64_t, NTValue>>> nt_timeseries;

    for (const auto& record : reader) {
        if (record.type != wpi::log::RecordType::kData) continue;

        NTValue value;
        if (!decode_record(record, reader.GetStart(record.name), value)) continue;

        nt_timeseries[std::string(record.name)].emplace_back(record.timestamp, value);
    }

    double delta_rot_threshold = 0.05;
    for (auto& [name, series] : nt_timeseries) {
        for (auto& [timestamp, val] : series) {
            if (std::holds_alternative<frc::Pose3d>(val)) {
                auto pose = std::get<frc::Pose3d>(val);
                auto rpy = pose.Rotation().ToYawPitchRoll();
                if (std::abs(rpy[0]) > delta_rot_threshold) {
                    std::cout << "Yaw exceeded threshold at " << timestamp << "\n";
                }
            }
        }
    }

    return 0;
}