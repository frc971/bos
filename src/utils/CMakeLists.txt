add_library(utils timer.cc nt_utils.cc camera_utils.cc log.cc constants_from_json.cc transform.cc wpi_log_sink.cc) 
target_link_libraries(utils PUBLIC wpilibc nlohmann_json::nlohmann_json absl::log absl::check absl::flags absl::flags_parse apriltag ntcore)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(utils PUBLIC pch.h)
endif()
