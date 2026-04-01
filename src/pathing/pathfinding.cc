#include "pathfinding.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace pathing {

auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
         Point end_point) -> std::vector<Node> {

  int sx = start_point.x;
  int sy = start_point.y;

  Node* start = &field[sy][sx];
  start->visited = true;
  start->cost = 0;

  std::deque<Node*> queue;
  queue.push_back(start);

  Node end = {.x = end_point.x, .y = end_point.y};

  bool path_completed = false;

  std::vector<std::pair<int, int>> dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  while (!path_completed) {

    Node current = *queue.front();
    int cx = current.x;
    int cy = current.y;
    current = field[cy][cx];
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

              queue.push_back(neighbor);
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

  Node* rcurrent = &field[end.x][end.y];
  std::vector<Node> rpath = {};
  while (rcurrent->x != start->x && rcurrent->y != start->y) {
    rpath.push_back(*rcurrent);
    Node* temp = rcurrent;
    rcurrent = temp->parent;
    field[rcurrent->y][rcurrent->x].path = true;
  }
  std::reverse(rpath.begin(), rpath.end());

  return rpath;
}

}  // namespace pathing
