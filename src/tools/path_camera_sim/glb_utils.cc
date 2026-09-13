#include "src/tools/path_camera_sim/glb_utils.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace path_camera_sim {
namespace {

constexpr std::array<char, 4> kGlbMagic{'g', 'l', 'T', 'F'};
constexpr uint32_t kGlbVersion = 2;
constexpr uint32_t kJsonChunkType = 0x4E4F534A;

struct GlbChunk {
  uint32_t type;
  std::vector<char> bytes;
};

struct GlbFile {
  std::vector<GlbChunk> chunks;
};

template <typename T>
auto ReadScalar(std::istream& input) -> T {
  T value{};
  input.read(reinterpret_cast<char*>(&value), sizeof(value));
  if (!input) {
    throw std::runtime_error("Unexpected end of GLB file");
  }
  return value;
}

template <typename T>
void WriteScalar(std::ostream& output, T value) {
  output.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

auto ReadGlb(const std::filesystem::path& path) -> GlbFile {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open GLB: " + path.string());
  }

  std::array<char, 4> magic{};
  input.read(magic.data(), magic.size());
  const uint32_t version = ReadScalar<uint32_t>(input);
  const uint32_t total_length = ReadScalar<uint32_t>(input);
  if (magic != kGlbMagic || version != kGlbVersion) {
    throw std::runtime_error("Unsupported GLB header: " + path.string());
  }

  GlbFile glb;
  uint32_t consumed = 12;
  while (consumed < total_length) {
    const uint32_t chunk_length = ReadScalar<uint32_t>(input);
    const uint32_t chunk_type = ReadScalar<uint32_t>(input);
    GlbChunk chunk{.type = chunk_type,
                   .bytes = std::vector<char>(chunk_length)};
    input.read(chunk.bytes.data(), chunk.bytes.size());
    if (!input) {
      throw std::runtime_error("Truncated GLB chunk: " + path.string());
    }
    consumed += 8 + chunk_length;
    glb.chunks.emplace_back(std::move(chunk));
  }
  if (consumed != total_length ||
      input.peek() != std::ifstream::traits_type::eof()) {
    throw std::runtime_error("Invalid GLB length: " + path.string());
  }
  return glb;
}

auto JsonChunk(GlbFile& glb) -> GlbChunk& {
  for (auto& chunk : glb.chunks) {
    if (chunk.type == kJsonChunkType) {
      return chunk;
    }
  }
  throw std::runtime_error("GLB contains no JSON chunk");
}

auto ParseJsonChunk(const GlbChunk& chunk) -> nlohmann::json {
  std::string text(chunk.bytes.begin(), chunk.bytes.end());
  while (!text.empty() && (text.back() == '\0' || text.back() == ' ')) {
    text.pop_back();
  }
  return nlohmann::json::parse(text);
}

void WriteGlb(const std::filesystem::path& path, GlbFile glb,
              const nlohmann::json& document) {
  std::string json_text = document.dump();
  while (json_text.size() % 4 != 0) {
    json_text.push_back(' ');
  }
  auto& json_chunk = JsonChunk(glb);
  json_chunk.bytes.assign(json_text.begin(), json_text.end());

  uint64_t total_length = 12;
  for (const auto& chunk : glb.chunks) {
    total_length += 8 + chunk.bytes.size();
  }
  if (total_length > UINT32_MAX) {
    throw std::runtime_error("GLB is too large to write");
  }

  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("Unable to write GLB: " + path.string());
  }
  output.write(kGlbMagic.data(), kGlbMagic.size());
  WriteScalar<uint32_t>(output, kGlbVersion);
  WriteScalar<uint32_t>(output, static_cast<uint32_t>(total_length));
  for (const auto& chunk : glb.chunks) {
    WriteScalar<uint32_t>(output, static_cast<uint32_t>(chunk.bytes.size()));
    WriteScalar<uint32_t>(output, chunk.type);
    output.write(chunk.bytes.data(), chunk.bytes.size());
  }
  if (!output) {
    throw std::runtime_error("Failed while writing GLB: " + path.string());
  }
}

auto RootMeshName(const std::filesystem::path& path) -> std::string {
  const auto json = ReadGlbJson(path);
  const auto& scenes = json.at("scenes");
  const size_t scene_index = json.value("scene", 0U);
  const auto& roots = scenes.at(scene_index).at("nodes");
  if (roots.size() != 1) {
    throw std::runtime_error(
        "Gamepiece GLB must contain exactly one root node: " + path.string());
  }
  return json.at("nodes")
      .at(roots.at(0).get<size_t>())
      .at("name")
      .get<std::string>();
}

}  // namespace

auto ReadGlbJson(const std::filesystem::path& path) -> nlohmann::json {
  auto glb = ReadGlb(path);
  return ParseJsonChunk(JsonChunk(glb));
}

auto PruneStagedGamepieces(const std::filesystem::path& field_glb,
                           const std::filesystem::path& field_config,
                           const std::filesystem::path& field_directory,
                           const std::filesystem::path& output_glb)
    -> PruneResult {
  std::ifstream config_stream(field_config);
  if (!config_stream) {
    throw std::runtime_error("Unable to open field config: " +
                             field_config.string());
  }
  nlohmann::json config;
  config_stream >> config;

  std::unordered_map<std::string, size_t> staged_counts;
  const auto& gamepieces = config.at("gamePieces");
  for (size_t i = 0; i < gamepieces.size(); ++i) {
    const auto model_path =
        field_directory / ("model_" + std::to_string(i) + ".glb");
    const std::string root_name = RootMeshName(model_path);
    const size_t expected = gamepieces.at(i).at("stagedObjects").size();
    if (!staged_counts.emplace(root_name, expected).second) {
      throw std::runtime_error(
          "Gamepiece GLBs use a duplicate root node name: " + root_name);
    }
  }

  auto glb = ReadGlb(field_glb);
  auto document = ParseJsonChunk(JsonChunk(glb));
  std::unordered_map<std::string, size_t> removed_by_name;
  for (auto& node : document.at("nodes")) {
    if (!node.contains("name") || !node.contains("mesh")) {
      continue;
    }
    const std::string name = node.at("name").get<std::string>();
    if (staged_counts.contains(name)) {
      node.erase("mesh");
      ++removed_by_name[name];
    }
  }

  size_t expected_total = 0;
  size_t removed_total = 0;
  for (const auto& [name, expected] : staged_counts) {
    const size_t removed = removed_by_name[name];
    if (removed != expected) {
      throw std::runtime_error("Staged node count mismatch for '" + name +
                               "': expected " + std::to_string(expected) +
                               ", found " + std::to_string(removed));
    }
    expected_total += expected;
    removed_total += removed;
  }

  WriteGlb(output_glb, std::move(glb), document);
  return {.removed_nodes = removed_total, .expected_nodes = expected_total};
}

}  // namespace path_camera_sim
