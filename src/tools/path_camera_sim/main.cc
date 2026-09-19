#include <unistd.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
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

constexpr char kXvfbRetryEnvironment[] = "PATH_CAMERA_SIM_XVFB_RETRY";

auto RetryUnderXvfb(const std::vector<std::string>& original_arguments) -> int {
  if (std::getenv(kXvfbRetryEnvironment) != nullptr) {
    return -1;
  }

  std::vector<std::string> command{"xvfb-run", "-a"};
  command.insert(command.end(), original_arguments.begin(),
                 original_arguments.end());
  std::vector<char*> command_arguments;
  command_arguments.reserve(command.size() + 1);
  for (auto& argument : command) {
    command_arguments.push_back(argument.data());
  }
  command_arguments.push_back(nullptr);

  if (setenv(kXvfbRetryEnvironment, "1", 1) != 0) {
    std::cerr << "path_camera_sim: unable to prepare Xvfb retry: "
              << std::strerror(errno) << '\n';
    return 1;
  }
  std::cerr << "path_camera_sim: no usable display; retrying under Xvfb\n";
  std::cout.flush();
  execvp(command.front().c_str(), command_arguments.data());
  std::cerr << "path_camera_sim: could not launch xvfb-run: "
            << std::strerror(errno)
            << " (install Xvfb or configure a working display)\n";
  return 1;
}

auto Resolve(const std::filesystem::path& root, const std::string& path)
    -> std::filesystem::path {
  const std::filesystem::path value(path);
  return value.is_absolute() ? value : root / value;
}

}  // namespace

auto main(int argc, char* argv[]) -> int {
  std::vector<std::string> original_arguments;
  original_arguments.reserve(argc);
  for (int i = 0; i < argc; ++i) {
    original_arguments.emplace_back(argv[i]);
  }
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
    constexpr std::string_view kDisplayError =
        "Open3D could not create an OpenGL window";
    if (std::string_view(error.what()).starts_with(kDisplayError)) {
      const int retry_result = RetryUnderXvfb(original_arguments);
      if (retry_result >= 0) {
        return retry_result;
      }
    }
    std::cerr << "path_camera_sim: " << error.what() << '\n';
    return 1;
  }
}
