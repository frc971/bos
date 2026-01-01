add_library(utils timer.cc nt_utils.cc camera_utils.cc log.cc) 
target_link_libraries(utils PUBLIC wpilibc nlohmann_json::nlohmann_json absl::log absl::check)
