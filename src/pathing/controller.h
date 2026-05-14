#pragma once

#include <string>
namespace pathing {

auto RunController(const std::string& navgrid_path, bool verbose) -> void;

}  // namespace pathing