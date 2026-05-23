#pragma once

#include <stop_token>
#include <string>
namespace pathing {

auto RunController(
    const std::stop_token& stop_token,
    const std::string& navgrid_path = "/root/bos/constants/navgrid.json",
    bool verbose = false) -> void;

}  // namespace pathing
