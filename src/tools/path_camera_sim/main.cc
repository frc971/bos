#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>

#include "src/tools/path_camera_sim/simulation.h"

ABSL_FLAG(std::string, auto_name, "Corner",
          "PathPlanner auto name (without .auto)");
ABSL_FLAG(std::string, pathplanner_dir, "paths/pathplanner",
          "Directory containing PathPlanner autos, paths, and settings.json");
ABSL_FLAG(std::string, camera, "second_bot_left",
          "Camera name from camera_constants.json");
ABSL_FLAG(std::string, camera_constants, "constants/camera_constants.json",
          "Camera constants JSON path");
ABSL_FLAG(std::string, field_dir, "field-cad",
          "AdvantageScope field asset directory");
ABSL_FLAG(std::string, gamepieces,
          "src/tools/path_camera_sim/testdata/corner_gamepieces.json",
          "Static WPILib-coordinate gamepiece poses JSON");
ABSL_FLAG(std::string, output_dir, "sim-output/Corner",
          "Output directory for PNG frames and manifest.json");
ABSL_FLAG(double, fps, 10.0, "Trajectory sampling rate in frames per second");
ABSL_FLAG(bool, apply_distortion, true,
          "Apply the calibrated lens distortion to rendered pinhole images");

namespace {

auto Resolve(const std::filesystem::path& root, const std::string& path)
    -> std::filesystem::path {
  const std::filesystem::path value(path);
  return value.is_absolute() ? value : root / value;
}

}  // namespace

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);
  try {
    const std::filesystem::path root(BOS_SOURCE_DIR);
    path_camera_sim::SimulationConfig config{
        .auto_name = absl::GetFlag(FLAGS_auto_name),
        .camera_name = absl::GetFlag(FLAGS_camera),
        .repository_root = root,
        .pathplanner_directory =
            Resolve(root, absl::GetFlag(FLAGS_pathplanner_dir)),
        .camera_constants_path =
            Resolve(root, absl::GetFlag(FLAGS_camera_constants)),
        .field_directory = Resolve(root, absl::GetFlag(FLAGS_field_dir)),
        .gamepieces_path = Resolve(root, absl::GetFlag(FLAGS_gamepieces)),
        .output_directory = Resolve(root, absl::GetFlag(FLAGS_output_dir)),
        .frames_per_second = absl::GetFlag(FLAGS_fps),
        .apply_distortion = absl::GetFlag(FLAGS_apply_distortion)};
    std::vector<const char*> open3d_arguments;
    open3d_arguments.reserve(argc);
    for (int i = 0; i < argc; ++i) {
      open3d_arguments.push_back(argv[i]);
    }
    path_camera_sim::RunSimulation(argc, open3d_arguments.data(), config);
    std::cout << "Simulation complete: " << config.output_directory << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "path_camera_sim: " << error.what() << '\n';
    return 1;
  }
}
