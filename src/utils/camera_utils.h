#include <string>
#include "nlohmann/json.hpp"

namespace utils {
nlohmann::json read_intrinsics(std::string path);
nlohmann::json read_extrinsics(std::string path);
} // utils
