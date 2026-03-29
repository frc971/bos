#include "src/pathing/pathing.h"
#include <absl/log/log.h>
#include <units/angle.h>
#include <units/length.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <queue>
#include <vector>

namespace pathing {

constexpr uchar WALL = 0;

auto InitializeGrid(const std::vector<std::vector<bool>>& gridData) -> cv::Mat {
  int GRID_H = gridData.size();
  int GRID_W = gridData[0].size();
  cv::Mat grid(GRID_H, GRID_W, CV_8UC1);
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      grid.at<uchar>(y, x) = gridData[y][x] ? 0 : 255;
    }
  }
  return grid;
}

auto BFS(const cv::Mat& grid, cv::Point2i start, cv::Point2i target, bool verbose)
    -> std::vector<cv::Point2i> {
  if (verbose) {
    std::cout << "BFS: Grid size: " << grid.cols << "x" << grid.rows
              << std::endl;
    std::cout << "BFS: Start (" << start.x << ", " << start.y
              << ") value: " << (int)grid.at<uchar>(start.y, start.x)
              << std::endl;
    std::cout << "BFS: Target (" << target.x << ", " << target.y
              << ") value: " << (int)grid.at<uchar>(target.y, target.x)
              << std::endl;
  }

  if (grid.at<uchar>(start.y, start.x) == 0) {
    std::cerr << "BFS: Start position is blocked!" << std::endl;
    return {};
  }
  if (grid.at<uchar>(target.y, target.x) == 0) {
    std::cerr << "BFS: Target position is blocked!" << std::endl;
    return {};
  }

  std::vector<std::vector<bool>> visited(grid.rows,
                                         std::vector<bool>(grid.cols));
  std::vector<std::vector<cv::Point2i>> parents(grid.rows,
                                             std::vector<cv::Point2i>(grid.cols));

  std::queue<cv::Point2i> queue;

  visited[start.y][start.x] = true;
  queue.push(start);

  const std::array<cv::Point2i, 8> dirs = {
      {{0, -1}, {0, 1}, {-1, 0}, {1, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};

  while (!queue.empty()) {
    cv::Point2i current = queue.front();
    queue.pop();

    if (current == target) {
      std::vector<cv::Point2i> path;
      cv::Point2i node = target;
      while (node != start) {
        path.push_back(node);
        node = parents[node.y][node.x];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const cv::Point2i& d : dirs) {
      int nx = current.x + d.x;
      int ny = current.y + d.y;

      if (nx < 0 || nx >= grid.cols || ny < 0 || ny >= grid.rows) {
        continue;
      }

      if (grid.at<uchar>(ny, nx) == WALL || visited[ny][nx]) {
        continue;
      }

      visited[ny][nx] = true;
      parents[ny][nx] = current;
      queue.push({nx, ny});
    }
  }

  return {};
}


auto ClampedUniformKnotVector(size_t num_control_points, uint degree)
    -> std::vector<double> {
  std::vector<double> knots;

  for (size_t i = 0; i <= degree; ++i) {
    knots.push_back(0.0);
  }

  int middle = num_control_points - degree - 1;
  for (int i = 1; i <= middle; ++i) {
    knots.push_back((double)i / (middle + 1));
  }

  for (size_t i = 0; i <= degree; ++i) {
    knots.push_back(1.0);
  }

  return knots;
}

auto BasisFunction(size_t index, uint degree, double t,
                   const std::vector<double>& knots) -> double {
  if (degree == 0) {
    return knots[index] <= t && t < knots[index + 1] ? 1 : 0;
  }

  double weight = 0.0;

  double denom1 = knots[index + degree] - knots[index];
  if (denom1 != 0) {
    weight += ((t - knots[index]) / denom1) *
              BasisFunction(index, degree - 1, t, knots);
  }
  double denom2 = knots[index + degree + 1] - knots[index + 1];
  if (denom2 != 0) {
    weight += ((knots[index + degree + 1] - t) / denom2) *
              BasisFunction(index + 1, degree - 1, t, knots);
  }
  return weight;
}

auto GetSplinePoint(double t, const std::vector<cv::Point2i>& points,
                    const std::vector<double>& knots, uint degree)
    -> frc::Translation2d {
  double px = 0.0, py = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    double weight = BasisFunction(i, degree, t, knots);
    px += points[i].x * weight;
    py += points[i].y * weight;
  }
  return {units::meter_t{px}, units::meter_t{py}};
}

auto CreateSpline(cv::Mat& grid, int start_x, int start_y, int target_x,
                  int target_y) -> std::vector<frc::Translation2d> {
  std::vector<cv::Point2i> control_points = BFS(grid, {start_x, start_y}, {target_x, target_y});
  if (control_points.empty()) {
    return {};
  }

  if (control_points.size() < 4) {
    LOG(WARNING) << "Too few points to generic cubic spline";
    return {};
  }

  std::vector<double> knots = ClampedUniformKnotVector(control_points.size(), 3);

  std::vector<frc::Translation2d> trajectory;
  int resolution = 200;
  for (int i = 0; i < resolution; ++i) {
    double t = (double)i / resolution;
    const frc::Translation2d point = GetSplinePoint(t, control_points, knots, 3);
    trajectory.push_back(point);
  }

  return trajectory;
}

}  // namespace pathing