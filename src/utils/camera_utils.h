#include <string>
#include "nlohmann/json.hpp"

namespace utils {
auto read_intrinsics(const std::string& path) -> nlohmann::json;
auto read_extrinsics(const std::string& path) -> nlohmann::json;
}  // namespace utils
