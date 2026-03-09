#include "src/camera/camera_constants.h"
namespace camera {

auto GetCameraConstants(const std::string& path) -> camera_constants_t {
  std::map<std::string, camera_constant_t> camera_constants;
  std::ifstream f(path);
  PCHECK(f) << "Failed to read camera constants json: " << path;

  nlohmann::json json;
  f >> json;

  const auto& camera_configs = json.at("cameras");
  for (const auto& camera_config : camera_configs) {
    if (camera_config.is_null()) {
      LOG(WARNING) << "Found a null camera config";
      continue;
    }
    if (!camera_config.contains("name") || camera_config["name"].is_null()) {
      LOG(WARNING) << "Could not find name in camera config";
      continue;
    }
    if (camera_constants.contains(camera_config["name"])) {
      LOG(WARNING) << "Duplicate cameras";
      continue;
    }

    camera_constant_t camera_constant{
        .name = camera_config.value("name", std::string{})};

    if (camera_config.contains("pipeline") &&
        !camera_config["pipeline"].is_null()) {
      camera_constant.pipeline = camera_config["pipeline"];
    }
    if (camera_config.contains("intrinsics_path") &&
        !camera_config["intrinsics_path"].is_null()) {
      camera_constant.intrinsics_path = camera_config["intrinsics_path"];
    }
    if (camera_config.contains("extrinsics_path") &&
        !camera_config["extrinsics_path"].is_null()) {
      camera_constant.extrinsics_path = camera_config["extrinsics_path"];
    }
    if (camera_config.contains("backlight") &&
        !camera_config["backlight"].is_null()) {
      camera_constant.backlight = camera_config["backlight"];
    }
    if (camera_config.contains("frame_width") &&
        !camera_config["frame_width"].is_null()) {
      camera_constant.frame_width = camera_config["frame_width"];
    }
    if (camera_config.contains("frame_height") &&
        !camera_config["frame_height"].is_null()) {
      camera_constant.frame_height = camera_config["frame_height"];
    }
    if (camera_config.contains("fps") && !camera_config["fps"].is_null()) {
      camera_constant.fps = camera_config["fps"];
    }
    if (camera_config.contains("exposure") &&
        !camera_config["exposure"].is_null()) {
      camera_constant.exposure = camera_config["exposure"];
    }
    if (camera_config.contains("brightness") &&
        !camera_config["brightness"].is_null()) {
      camera_constant.sharpness = camera_config["brightness"];
    }
    if (camera_config.contains("sharpness") &&
        !camera_config["sharpness"].is_null()) {
      camera_constant.sharpness = camera_config["sharpness"];
    }
    camera_constants.insert({camera_constant.name, camera_constant});
  }
  return camera_constants;
}
}  // namespace camera
