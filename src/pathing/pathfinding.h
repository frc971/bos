#pragma once

#include <sys/types.h>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>

namespace pathing {

struct Node {
  uint x, y;
  double cost = INFINITY;
  bool visited = false;
  bool obstacle = false;
  char readable;
  bool path = false;
  Node* parent = nullptr;

  auto operator==(const Node& other) const -> bool {
    return x == other.x && y == other.y;
  }
};

struct Point {
  uint x, y;
};

auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
         Point end_point) -> std::vector<Node>;

}  // namespace pathing
