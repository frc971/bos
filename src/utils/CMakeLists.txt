add_library(utils timer.cc nt_utils.cc camera_utils.cc log.cc intrinsics_from_json.cc extrinsics_from_json.cc) 
target_link_libraries(utils PUBLIC wpilibc nlohmann_json::nlohmann_json absl::log absl::check)
target_precompile_headers(utils PRIVATE pch.h)
