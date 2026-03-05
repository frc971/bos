#pragma once
#include <string>
#include <unordered_map>
#include "nlohmann/json.hpp"

namespace utils {
auto GetJson(const std::string& path) -> nlohmann::json;
auto GetJsonCache() -> const std::unordered_map<std::string, nlohmann::json>&;
}  // namespace utils
