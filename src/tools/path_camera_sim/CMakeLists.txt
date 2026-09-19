find_package(Open3D REQUIRED)

include(FetchContent)

set(PATHPLANNER_VERSION "2026.1.2")
set(PATHPLANNER_MAVEN_BASE
    "https://3015rangerrobotics.github.io/pathplannerlib/repo/com/pathplanner/lib/PathplannerLib-cpp/${PATHPLANNER_VERSION}")

FetchContent_Declare(
  pathplanner_headers
  URL "${PATHPLANNER_MAVEN_BASE}/PathplannerLib-cpp-${PATHPLANNER_VERSION}-headers.zip"
  URL_HASH SHA256=923a1b41a8476f6c80bdbd41f3431bcb29df4c18354014292f38cf0eaf45c894
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_Declare(
  pathplanner_arm64
  URL "${PATHPLANNER_MAVEN_BASE}/PathplannerLib-cpp-${PATHPLANNER_VERSION}-linuxarm64.zip"
  URL_HASH SHA256=4d42d0eb46f6310c0bb2891f0d32044d2b5e2ffe47f19139526d2512c4b17651
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  message(FATAL_ERROR "path_camera_sim currently supports the official linuxarm64 PathPlanner artifact only")
endif()

FetchContent_MakeAvailable(pathplanner_headers pathplanner_arm64)

set(PATHPLANNER_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/pathplanner_include")
file(MAKE_DIRECTORY "${PATHPLANNER_INCLUDE_DIR}/pathplanner")
file(CREATE_LINK "${pathplanner_headers_SOURCE_DIR}/lib"
  "${PATHPLANNER_INCLUDE_DIR}/pathplanner/lib" SYMBOLIC)

add_library(PathplannerLib SHARED IMPORTED GLOBAL)
set_target_properties(PathplannerLib PROPERTIES
  IMPORTED_LOCATION "${pathplanner_arm64_SOURCE_DIR}/arm64/shared/libPathplannerLib.so"
  INTERFACE_INCLUDE_DIRECTORIES "${PATHPLANNER_INCLUDE_DIR}"
)

add_library(path_camera_sim_lib
  gamepiece_config.cc
  glb_utils.cc
  simulation.cc
)
target_link_libraries(path_camera_sim_lib PUBLIC
  CameraConstants
  Open3D::Open3D
  PathplannerLib
  nlohmann_json::nlohmann_json
  opencv_core
  opencv_calib3d
  opencv_imgproc
  opencv_imgcodecs
  utils
  wpilibNewCommands
)
target_compile_definitions(path_camera_sim_lib PUBLIC BOS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(path_camera_sim main.cc)
target_link_libraries(path_camera_sim PRIVATE
  absl::flags
  absl::flags_parse
  path_camera_sim_lib
)

add_executable(generate_fuel_config generate_fuel_config.cc)
target_link_libraries(generate_fuel_config PRIVATE
  absl::flags
  absl::flags_parse
  path_camera_sim_lib
)

add_custom_command(TARGET path_camera_sim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:PathplannerLib>"
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<TARGET_FILE_NAME:PathplannerLib>"
)

add_executable(path_camera_sim_test simulation_test.cc)
target_link_libraries(path_camera_sim_test PRIVATE
  GTest::gtest_main
  path_camera_sim_lib
)
include(GoogleTest)
gtest_discover_tests(path_camera_sim_test)
