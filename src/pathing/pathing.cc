#include "src/pathing/pathing.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <queue>



auto initializeGrid(const std::vector<std::vector<bool>>& gridData) -> cv::Mat {

  int H = gridData.size();
  int W = gridData[0].size();

  cv::Mat grid(H, W, CV_8UC1);

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      grid.at<uchar>(y, x) = gridData[y][x] ? 0 : 255;

  return grid;
}

auto BFS(const cv::Mat& grid,
                                     std::pair<int, int> start,
                                     std::pair<int, int> target) {

  if (grid.at<uchar>(start.second, start.first) == 0)
    return {};
  if (grid.at<uchar>(target.second, target.first) == 0)
    return {};

  cv::Mat visited(grid.rows, grid.cols, CV_8UC1, cv::Scalar(0));
  cv::Mat parent(grid.rows, grid.cols, CV_32SC2, cv::Scalar(-1, -1));

  std::queue<std::pair<int, int>> q;

  visited.at<uchar>(start.second, start.first) = 1;
  q.push(start);

  std::vector<std::pair<int, int>> dirs = {{0, -1},  {0, 1},  {-1, 0}, {1, 0},
                                           {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

  while (!q.empty()) {

    auto cur = q.front();
    q.pop();

    if (cur == target) {
      std::vector<std::pair<int, int>> path;
      auto n = target;

      while (true) {
        path.push_back(n);
        cv::Vec2i p = parent.at<cv::Vec2i>(n.second, n.first);
        if (p[0] == -1)
          break;
        n = {p[0], p[1]};
      }

      std::reverse(path.begin(), path.end());
      return path;
    }

    for (auto& d : dirs) {

      int nx = cur.first + d.first;
      int ny = cur.second + d.second;

      if (nx < 0 || ny < 0 || nx >= grid.cols || ny >= grid.rows)
        continue;
      if (grid.at<uchar>(ny, nx) == 0)
        continue;

      if (d.first != 0 && d.second != 0) {
        if (grid.at<uchar>(cur.second, nx) == 0)
          continue;
        if (grid.at<uchar>(ny, cur.first) == 0)
          continue;
      }

      if (visited.at<uchar>(ny, nx))
        continue;

      visited.at<uchar>(ny, nx) = 1;
      parent.at<cv::Vec2i>(ny, nx) = cv::Vec2i(cur.first, cur.second);

      q.push({nx, ny});
    }
  }

  return {};
}

auto createSpline(cv::Mat& grid, int sx, int sy, int tx,
                                      int ty, double nodeSize) -> std::vector<frc::Pose2d> {

  auto path = BFS(grid, {sx, sy}, {tx, ty});

  if (path.empty())
    return {};

  std::vector<std::pair<double, double>> pts;
  pts.reserve(path.size());

  for (auto& p : path) {
    pts.emplace_back(p.first * nodeSize + nodeSize / 2,
                     p.second * nodeSize + nodeSize / 2);
  }

  std::vector<frc::Pose2d> traj;

  double spacing = 0.05;

  for (size_t i = 1; i < pts.size(); i++) {

    double dx = pts[i].first - pts[i - 1].first;
    double dy = pts[i].second - pts[i - 1].second;

    double len = std::hypot(dx, dy);

    int steps = std::max(1, (int)(len / spacing));

    for (int s = 0; s < steps; s++) {

      double t = (double)s / steps;

      double x = pts[i - 1].first + dx * t;
      double y = pts[i - 1].second + dy * t;

      traj.push_back(frc::Pose2d(units::meter_t{x}, units::meter_t{y},
                                 units::radian_t{0}));
    }
  }

  traj.push_back(frc::Pose2d(units::meter_t{pts.back().first},
                             units::meter_t{pts.back().second},
                             units::radian_t{0}));

  return traj;
}