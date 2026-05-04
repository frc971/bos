#include "pathfinding.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <utility>
#include <vector>
#include "src/utils/log.h"

namespace pathing {

auto BFSFirstFreeCell(std::vector<std::vector<Node>>& field, Point start_point)
    -> Node {
  int sx = start_point.x;
  int sy = start_point.y;

  field[sy][sx].visited = true;

  std::deque<Point> queue;
  queue.push_back(start_point);

  std::vector<std::pair<int, int>> dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  while (!queue.empty()) {
    const Point current_point = queue.front();
    queue.pop_front();

    Node& current = field[current_point.y][current_point.x];

    for (auto [dy, dx] : dirs) {
      int nx = current.x + dx;
      int ny = current.y + dy;

      if (nx >= 0 && nx < (int)field[0].size() && ny >= 0 &&
          ny < (int)field.size()) {
        if (!field[ny][nx].visited) {
          field[ny][nx].visited = true;
          if (!field[ny][nx].obstacle) {
            return field[ny][nx];
          }
          queue.push_back({.x = (uint)nx, .y = (uint)ny});
        }
      }
    }
  }

  return field[sy][sx];
}

auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
         Point end_point) -> std::vector<Node> {

  int sx = start_point.x;
  int sy = start_point.y;
  if (field[sy][sx].obstacle) {
    Node adjusted = BFSFirstFreeCell(field, start_point);
    sx = adjusted.x;
    sy = adjusted.y;
  }

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
      uint nx = current.x + dir.first;
      uint ny = current.y + dir.second;

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
  if (rcurrent != nullptr) {
    rpath.push_back(*rcurrent);
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
