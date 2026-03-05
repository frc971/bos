#include "src/utils/camera_utils.h"
#include <fstream>
#include <unordered_map>

namespace utils {
namespace {
std::unordered_map<std::string, nlohmann::json> json_cache;
}

auto GetJson(const std::string& path) -> nlohmann::json {
  auto [it, inserted] = json_cache.emplace(path, nlohmann::json{});
  if (inserted) {
    std::ifstream f(path);
    if (f) f >> it->second;
  }
  return it->second;
}

auto GetJsonCache() -> const std::unordered_map<std::string, nlohmann::json>& {
  return json_cache;
}
}  // namespace utils
