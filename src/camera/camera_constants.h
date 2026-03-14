#pragma once

#include <array>
#include "src/utils/log.h"
#include "src/utils/pch.h"
namespace camera {

using camera_constant_t = struct CameraConstant {
  std::string name;
  std::optional<std::string> pipeline = std::nullopt;
  std::optional<std::string> intrinsics_path = std::nullopt;
  std::optional<std::string> extrinsics_path = std::nullopt;
  std::optional<double> backlight = std::nullopt;
  std::optional<double> frame_width = std::nullopt;
  std::optional<double> frame_height = std::nullopt;
  std::optional<double> fps = std::nullopt;
  std::optional<double> exposure = std::nullopt;  // Nullopt = auto exposure
  std::optional<double> brightness = std::nullopt;
  std::optional<double> sharpness = std::nullopt;

  auto operator==(const CameraConstant& other) const -> bool {
    return name == other.name;
  }

  friend auto operator<<(std::ostream& os, const CameraConstant& c)
      -> std::ostream& {
    os << "pipeline: " << c.pipeline.value_or("NO PIPELINE VALUE")
       << "\tintrinsics_path: " << c.intrinsics_path.value_or("NO INTRINSICS")
       << "\textrinsics_path: " << c.extrinsics_path.value_or("NO EXTRINSICS")
       << "\tname: " << c.name << '\n';

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

using camera_constants_t = std::unordered_map<std::string, camera_constant_t>;

auto GetCameraConstants(const std::string& path) -> camera_constants_t;

auto GetCameraConstants() -> camera_constants_t;

}  // namespace camera
