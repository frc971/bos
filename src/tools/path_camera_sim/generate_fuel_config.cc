#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>

#include "src/tools/path_camera_sim/gamepiece_config.h"

ABSL_FLAG(double, entropy, 0.0, "Fuel disorder from 0 (staged) to 1 (maximum)");
ABSL_FLAG(uint32_t, seed, 0, "Random seed used for reproducible layouts");
ABSL_FLAG(std::string, field_dir, "field-cad",
          "AdvantageScope field asset directory");
ABSL_FLAG(std::string, output, "sim-output/fuel_gamepieces.json",
          "Generated gamepiece JSON path");

namespace {

auto Resolve(const std::filesystem::path& root, const std::string& path)
    -> std::filesystem::path {
  const std::filesystem::path value(path);
  return value.is_absolute() ? value : root / value;
}

}  // namespace

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);
  try {
    const std::filesystem::path root(BOS_SOURCE_DIR);
    const path_camera_sim::FuelGenerationOptions options{
        .entropy = absl::GetFlag(FLAGS_entropy),
        .seed = absl::GetFlag(FLAGS_seed)};
    const auto output = Resolve(root, absl::GetFlag(FLAGS_output));
    const auto gamepieces = path_camera_sim::GenerateFuelGamepieces(
        Resolve(root, absl::GetFlag(FLAGS_field_dir)), options);
    path_camera_sim::WriteGamepieceConfig(output, gamepieces, options);
    std::cout << "Wrote " << gamepieces.size() << " Fuel poses to " << output
              << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "generate_fuel_config: " << error.what() << '\n';
    return 1;
  }
}
