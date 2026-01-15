#include "src/gamepiece/gamepiece.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/utils/camera_utils.h"
#include "src/yolo/model_constants.h"
#include "src/yolo/yolo.h"

auto main() -> int {
  yolo::ModelInfo model_info = yolo::models[yolo::Model::COLOR];
  yolo::Yolo color_model(model_info.path, model_info.color);

  std::thread usb0_gamepiece_thread(
      gamepiece::run_gamepiece_detect_no_img, std::ref(color_model),
      std::ref(model_info.class_names));

  usb0_gamepiece_thread.join();
}
