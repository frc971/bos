#include "src/pathing/pathing.h"
#include <frc/DataLogManager.h>
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <units/angle.h>
#include <units/length.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <queue>
#include <vector>

const int CELL_SIZE = 20;

auto initializeGrid(const std::vector<std::vector<bool>>& gridData) -> cv::Mat {
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

auto BFS(const cv::Mat& grid, std::pair<int, int> start,
         std::pair<int, int> target) -> std::vector<std::pair<int, int>> {
  std::cout << "BFS: Grid size: " << grid.cols << "x" << grid.rows << std::endl;
  std::cout << "BFS: Start (" << start.first << ", " << start.second 
            << ") value: " << (int)grid.at<uchar>(start.second, start.first) << std::endl;
  std::cout << "BFS: Target (" << target.first << ", " << target.second 
            << ") value: " << (int)grid.at<uchar>(target.second, target.first) << std::endl;

  if (grid.at<uchar>(start.second, start.first) == 0) {
    std::cerr << "BFS: Start position is blocked!" << std::endl;
    return {};
  }
  if (grid.at<uchar>(target.second, target.first) == 0) {
    std::cerr << "BFS: Target position is blocked!" << std::endl;
    return {};
  }

  cv::Mat visited(grid.rows, grid.cols, CV_8UC1, cv::Scalar(0));
  cv::Mat parent(grid.rows, grid.cols, CV_32SC2, cv::Scalar(-1, -1));

  std::queue<std::pair<int, int>> queue;

  visited.at<uchar>(start.second, start.first) = 1;
  queue.push(start);

  std::vector<std::pair<int, int>> dirs = {{0, -1},  {0, 1},  {-1, 0}, {1, 0},
                                           {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

  while (!queue.empty()) {
    auto current = queue.front();
    queue.pop();

    if (current == target) {
      std::vector<std::pair<int, int>> path;
      auto node = target;
      while (true) {
        path.push_back(node);
        cv::Vec2i p = parent.at<cv::Vec2i>(node.second, node.first);
        if (p[0] == -1) {
          break;
        }
        node = {p[0], p[1]};
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const std::pair<int, int>& d : dirs) {
      int nx = current.first + d.first;
      int ny = current.second + d.second;

      if (nx < 0 || nx >= grid.cols || ny < 0 || ny >= grid.rows) {
        continue;
      }

      if (grid.at<uchar>(ny, nx) == 0 || visited.at<uchar>(ny, nx) == 1) {
        continue;
      }

      visited.at<uchar>(ny, nx) = 1;
      parent.at<cv::Vec2i>(ny, nx) = cv::Vec2i(current.first, current.second);
      queue.push({nx, ny});
    }
  }

  return {};
}

auto constructLinePath(cv::Mat& canvas, std::vector<std::pair<int, int>> path)
    -> std::vector<std::pair<int, int>> {
  std::vector<std::pair<int, int>> controlPoints;
  for (size_t i = 0; i < path.size(); ++i) {
    int px = path[i].first * CELL_SIZE + (CELL_SIZE / 2);
    int py = path[i].second * CELL_SIZE + (CELL_SIZE / 2);

    controlPoints.emplace_back(px, py);

    if (i > 0) {
      int prevX = path[i - 1].first * CELL_SIZE + (CELL_SIZE / 2);
      int prevY = path[i - 1].second * CELL_SIZE + (CELL_SIZE / 2);
      cv::line(canvas, cv::Point(prevX, prevY), cv::Point(px, py), {0, 0, 0},
               2);
    }
  }
  return controlPoints;
}

auto clampedUniformKnotVector(double k, double p) -> std::vector<double> {
  std::vector<double> knots;
  int n = (int)k;
  int d = (int)p;

  for (int i = 0; i <= d; ++i) {
    knots.push_back(0.0);
  }

  int middle = n - d - 1;
  for (int i = 1; i <= middle; ++i) {
    knots.push_back((double)i / (middle + 1));
  }

  for (int i = 0; i <= d; ++i) {
    knots.push_back(1.0);
  }

  return knots;
}

auto basisFunction(double i, double p, double t,
                   const std::vector<double>& knots) -> double {
  int idx = static_cast<int>(i);
  int deg = static_cast<int>(p);

  if (deg == 0) {
    if (p == 0) {
      if (knots[i] <= t && t < knots[i + 1]) {
        return 1.0;
      }

      if (t == 0.0 && knots[i] == 0.0 && knots[i + 1] > 0.0) {
        return 1.0;
      }

      if (t == 1.0 && knots[i + 1] == 1.0 && knots[i] < 1.0) {
        return 1.0;
      }

      return 0.0;
    }
  }

  double weight = 0.0;

  double denom1 = knots[idx + deg] - knots[idx];
  if (denom1 != 0) {
    weight += ((t - knots[idx]) / denom1) * basisFunction(i, p - 1, t, knots);
  }
  double denom2 = knots[idx + deg + 1] - knots[idx + 1];
  if (denom2 != 0) {
    weight += ((knots[idx + deg + 1] - t) / denom2) *
              basisFunction(i + 1, p - 1, t, knots);
  }
  return weight;
}

auto getSplinePoint(double t, const std::vector<std::pair<int, int>>& points,
                    const std::vector<double>& knots, int p)
    -> std::pair<double, double> {
  double px = 0.0, py = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    double weight = basisFunction((double)i, (double)p, t, knots);
    px += points[i].first * weight;
    py += points[i].second * weight;
  }
  return {px, py};
}

auto createSpline(cv::Mat& grid, int start_x, int start_y, int target_x,
                  int target_y, double nodeSizeMeters)
    -> std::vector<frc::Pose2d> {
  auto path = BFS(grid, {start_x, start_y}, {target_x, target_y});
  if (path.empty()) {
    return {};
  }

  auto controlPoints = constructLinePath(grid, path);

  if (controlPoints.size() < 4) {
    return {};
  }

  auto knots = clampedUniformKnotVector(controlPoints.size(), 3);

  std::vector<frc::Pose2d> trajectory;
  int resolution = 200;
  for (int i = 0; i < resolution; ++i) {
    double t = (double)i / resolution;
    std::pair<double, double> point =
        getSplinePoint(t, controlPoints, knots, 3);
    double px = point.first / CELL_SIZE;
    double py = point.second / CELL_SIZE;
    trajectory.push_back(frc::Pose2d(units::meter_t{px * nodeSizeMeters},
                                     units::meter_t{py * nodeSizeMeters},
                                     units::radian_t{0.0}));
  }

  return trajectory;
}