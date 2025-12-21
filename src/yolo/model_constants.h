#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

namespace yolo {
struct Model {
  const std::string path;
  const std::vector<std::string> class_names;
  const bool color;

  Model(std::string p, std::vector<std::string> names, bool color)
      : path(std::move(p)), class_names(std::move(names)), color(color) {}
};

const std::map<std::string, Model> models{
    {
        "color",
        Model{"/bos/models/color.engine",
              std::vector<std::string>{"coral", "coral", "algae", "algae"},
              true},
    },
    {"gray", Model{"/bos/models/gray.engine",
                   std::vector<std::string>{"coral", "algae"}, true}}};
}  // namespace yolo
