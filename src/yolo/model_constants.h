#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

namespace yolo {
struct ModelInfo {
  const std::string path;
  const std::vector<std::string> class_names;
  const bool color;

  ModelInfo(std::string p, std::vector<std::string> names, bool color)
      : path(std::move(p)), class_names(std::move(names)), color(color) {}
};

enum Model { COLOR = 0, GRAY = 1, COUNT };

inline const ModelInfo models[Model::COUNT]{
    [Model::COLOR] =
        ModelInfo{"/bos/models/color.engine",
                  std::vector<std::string>{"coral", "coral", "algae", "algae"},
                  true},
    [Model::GRAY] =
        ModelInfo{"/bos/models/gray.engine",
                  std::vector<std::string>{"coral", "algae"}, true}};
} // namespace yolo
