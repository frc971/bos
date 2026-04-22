#include "pathfinding.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <utility>
#include <vector>
#include "src/utils/log.h"

namespace pathing {

auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
         Point end_point) -> std::vector<Node> {

  int sx = start_point.x;
  int sy = start_point.y;

  Node* start = &field[sy][sx];
  start->visited = true;
  start->cost = 0;

  std::deque<Point> queue;
  queue.push_back(start_point);

  Node end = {.x = end_point.x, .y = end_point.y};

  bool path_completed = false;

  std::vector<std::pair<int, int>> dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  while (!path_completed && !queue.empty()) {
    const Point current_point = queue.front();
    int cx = current_point.x;
    int cy = current_point.y;
    Node& current = field[cy][cx];
    queue.pop_front();

    for (std::pair<int, int> dir : dirs) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      if (nx >= 0 && nx < field[0].size()) {
        if (ny >= 0 && ny < field.size()) {
          if (!field[ny][nx].visited) {
            if (!field[ny][nx].obstacle) {
              Node* neighbor = &field[ny][nx];
              neighbor->x = nx;
              neighbor->y = ny;
              (abs(dir.first) == abs(dir.second))
                  ? neighbor->cost = current.cost + sqrt(2)
                  : neighbor->cost = current.cost + 1;
              neighbor->visited = true;
              neighbor->parent = &field[cy][cx];

              queue.push_back({.x = nx, .y = ny});
              if (ny == end.y && nx == end.x) {
                path_completed = true;

                break;
              }
            }
          }
        }
      }
    }

    if (path_completed) {
      break;
    }
  }

  if (!path_completed) {
    LOG(INFO) << "path couldn't be completed";
    return {};
  }

  Node* rcurrent = &field[end.y][end.x];
  std::vector<Node> rpath = {};
  while (rcurrent != nullptr &&
         !(rcurrent->x == start->x && rcurrent->y == start->y)) {
    rpath.push_back(*rcurrent);
    rcurrent = rcurrent->parent;
    if (rcurrent != nullptr) {
      field[rcurrent->y][rcurrent->x].path = true;
    }
  }
  std::reverse(rpath.begin(), rpath.end());

  std::cout << "BFS path (" << start_point.x << "," << start_point.y << ") -> ("
            << end_point.x << "," << end_point.y << "): ";
  for (const auto& node : rpath) {
    std::cout << "(" << node.x << "," << node.y << ") ";
  }
  std::cout << std::endl;

  return rpath;
}

}  // namespace pathing
