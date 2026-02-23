#pragma once

#include <array>
#include "src/utils/pch.h"
namespace camera {

using camera_constant_t = struct CameraConstant {
  std::string pipeline;
  std::string intrinsics_path;
  std::string extrinsics_path;
  std::string name;
  std::optional<double> backlight = std::nullopt;
  std::optional<double> frame_width = std::nullopt;
  std::optional<double> frame_height = std::nullopt;
  std::optional<double> fps = std::nullopt;
  std::optional<double> exposure = std::nullopt;  // Nullopt = auto exposure
  std::optional<double> brightness = std::nullopt;
  std::optional<double> sharpness = std::nullopt;

  friend auto operator<<(std::ostream& os, const CameraConstant& c)
      -> std::ostream& {
    os << "pipeline: " << c.pipeline
       << "\tintrinsics_path: " << c.intrinsics_path
       << "\textrinsics_path: " << c.extrinsics_path << "\tname: " << c.name
       << '\n';

    const auto print = [&](std::string_view label, const auto& opt) {
      if (opt)
        os << '\t' << label << ": " << *opt;
    };

    print("Backlight", c.backlight);
    print("Frame Width", c.frame_width);
    print("Frame Height", c.frame_height);
    print("Fps", c.fps);
    print("Exposure", c.exposure);
    print("Brightness", c.brightness);
    print("Sharpness", c.sharpness);
    os << std::endl;

    return os;
  }

  auto operator<(const CameraConstant& other) const -> bool {
    return name < other.name;
  }
};

enum Camera {
  IMX296_0,
  IMX296_1,

  FIDDLER_USB0,
  FIDDLER_USB1,

  TURRET_BOT_FRONT_RIGHT,
  TURRET_BOT_FRONT_LEFT,

  MAIN_ROBOT_FRONT_CAMERA,
  MAIN_ROBOT_LEFT_CAMERA,
  MAIN_ROBOT_RIGHT_CAMERA,

  DEV_ORIN,
  DUMMY_CAMERA,  // For tests such as solver_test.cc
  CAMERA_LENGTH,
};

inline const std::array<camera_constant_t, CAMERA_LENGTH> camera_constants =
    []() {
      std::array<camera_constant_t, CAMERA_LENGTH> arr{};
      const std::string path = "/bos/constants/camera_constants.json";
      std::ifstream f(path);
      if (!f) {
        std::cerr << "camera_constants: failed to open " << path << std::endl;
        return arr;
      }

      try {
        nlohmann::json j;
        f >> j;
        const auto& cams = j.at("cameras");
        for (size_t i = 0; i < cams.size() && i < CAMERA_LENGTH; ++i) {
          const auto& cj = cams.at(i);
          auto& c = arr[i];
          c.pipeline = cj.value("pipeline", std::string{});
          c.intrinsics_path = cj.value("intrinsics_path", std::string{});
          c.extrinsics_path = cj.value("extrinsics_path", std::string{});
          c.name = cj.value("name", std::string{});
          if (cj.contains("backlight") && !cj["backlight"].is_null())
            c.backlight = cj["backlight"].get<double>();
          if (cj.contains("frame_width") && !cj["frame_width"].is_null())
            c.frame_width = cj["frame_width"].get<double>();
          if (cj.contains("frame_height") && !cj["frame_height"].is_null())
            c.frame_height = cj["frame_height"].get<double>();
          if (cj.contains("fps") && !cj["fps"].is_null())
            c.fps = cj["fps"].get<double>();
          if (cj.contains("exposure") && !cj["exposure"].is_null())
            c.exposure = cj["exposure"].get<double>();
          if (cj.contains("brightness") && !cj["brightness"].is_null())
            c.brightness = cj["brightness"].get<double>();
          if (cj.contains("sharpness") && !cj["sharpness"].is_null())
            c.sharpness = cj["sharpness"].get<double>();
        }
      } catch (const std::exception& e) {
        std::cerr << "camera_constants: json parse error: " << e.what()
                  << std::endl;
      }

      return arr;
    }();

inline auto GetCameraConstants()
    -> const std::array<camera_constant_t, CAMERA_LENGTH>& {
  return camera_constants;
}
inline auto GetCameraConstant(Camera c) -> const camera_constant_t& {
  return camera_constants[static_cast<size_t>(c)];
}

}  // namespace camera
