#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <frc/geometry/Pose3d.h>

namespace path_camera_sim {

struct GamepiecePose {
  std::string type;
  frc::Pose3d pose;
};

struct FuelGenerationOptions {
  // Entropy is normalized: 0 reproduces the staged field model and 1 applies
  // the maximum spread and removal probability.
  double entropy = 0.0;
  uint32_t seed = 0;
};

auto ReadGamepieceConfig(const std::filesystem::path& path)
    -> std::vector<GamepiecePose>;

// Kept as the simulator-facing name for existing callers.
auto LoadGamepieces(const std::filesystem::path& path)
    -> std::vector<GamepiecePose>;

void WriteGamepieceConfig(const std::filesystem::path& path,
                          const std::vector<GamepiecePose>& gamepieces,
                          const FuelGenerationOptions& options);

// Reads the staged Fuel transforms from model.glb and config.json in the field
// directory, converts them to WPILib coordinates, and applies entropy.
auto GenerateFuelGamepieces(const std::filesystem::path& field_directory,
                            const FuelGenerationOptions& options)
    -> std::vector<GamepiecePose>;

}  // namespace path_camera_sim
