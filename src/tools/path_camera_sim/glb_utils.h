#pragma once

#include <filesystem>
#include <string>

#include "nlohmann/json.hpp"

namespace path_camera_sim {

struct PruneResult {
  size_t removed_nodes;
  size_t expected_nodes;
};

auto ReadGlbJson(const std::filesystem::path& path) -> nlohmann::json;

// Writes a copy of the field GLB with staged gamepiece mesh references removed.
// The source asset and binary mesh data are not modified.
auto PruneStagedGamepieces(const std::filesystem::path& field_glb,
                           const std::filesystem::path& field_config,
                           const std::filesystem::path& field_directory,
                           const std::filesystem::path& output_glb)
    -> PruneResult;

}  // namespace path_camera_sim
