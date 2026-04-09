#include <climits>
#include <string>
#include <vector>

#pragma once

namespace pathing {

struct Node {
  int x, y;
  int cost = INT_MAX;
  bool visited = false;
  bool obstacle = false;
  bool path = false;
  char readble;
  Node* parent = nullptr;

  auto operator==(const Node& other) const -> bool {
    return x == other.x && y == other.y;
  }
};

struct Point {
  int x, y;
};

auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
         Point end_point) -> std::vector<Node>;

}  // namespace pathing
