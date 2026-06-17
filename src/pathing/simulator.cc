#include <frc/geometry/Pose2d.h>
#include <cmath>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <utility>
#include <vector>
#include "src/pathing/pathfinding.h"
#include "src/pathing/splines.h"
#include "src/pathing/velocity_profile.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

namespace pathing {

int GRID_H;
int GRID_W;
int CELL_SIZE = 20;

auto getGrid() -> const std::vector<std::vector<Node>>& {
  static std::vector<std::vector<Node>> grid;
  if (grid.empty()) {
    std::ifstream file("/root/bos/constants/navgrid.json");
    if (!file.is_open()) {
      LOG(FATAL) << "Failed to open navgrid.json";
      return grid;
    }

    nlohmann::json data = nlohmann::json::parse(file);
    file.close();

    GRID_H = data["grid"].size();
    GRID_W = data["grid"][0].size();

    grid.resize(GRID_H, std::vector<Node>(GRID_W));
    for (int y = 0; y < GRID_H; ++y) {
      for (int x = 0; x < GRID_W; ++x) {
        grid[y][x].x = x;
        grid[y][x].y = y;
        grid[y][x].obstacle = data["grid"][y][x];
      }
    }
  }
  return grid;
}

auto getNodeSize() -> double {
  std::ifstream file("/root/bos/constants/navgrid.json");
  nlohmann::json data = nlohmann::json::parse(file);
  return data["nodeSizeMeters"];
}

auto toGridPoint(double xMeters, double yMeters, double nodeSizeMeters)
    -> Point {
  int gx =
      std::clamp(static_cast<int>(xMeters / nodeSizeMeters), 0, GRID_W - 1);
  int gy =
      std::clamp(static_cast<int>(yMeters / nodeSizeMeters), 0, GRID_H - 1);
  return Point{.x = static_cast<uint>(gx), .y = static_cast<uint>(gy)};
}

auto generateExpectedPath(Point start, Point end, double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  const auto& grid = getGrid();
  SplineResult result = CreateSpline(grid, start, end, nodeSizeMeters, 200);

  std::vector<std::pair<double, double>> positions;
  for (const auto& pt : result.points) {
    double px = (pt.X().value() / nodeSizeMeters) * CELL_SIZE;
    double py = (pt.Y().value() / nodeSizeMeters) * CELL_SIZE;
    positions.emplace_back(px, py);
  }
  return positions;
}

auto simulateRobotPath(Point start, Point end, double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  const auto& grid = getGrid();
  SplineResult result = CreateSpline(grid, start, end, nodeSizeMeters, 200);
  std::vector<std::pair<double, double>> velocity_profile =
      CreateVelocityProfile(result);

  std::vector<std::pair<double, double>> trajectory;
  if (result.points.empty() || velocity_profile.empty()) {
    return trajectory;
  }

  const double dt = 0.02;             // 20 ms
  const double k = 0.15;              // noise fraction
  const double kp = 10;               // pull coeficcient
  const int maxTicks = 2000;          // don't go infinite loop
  const double errorThreshold = 0.4;  // max error before replanning in m

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distr(-1.0, 1);

  // current
  double x = result.points.front().X().value();
  double y = result.points.front().Y().value();

  //target
  const double tx = result.points.back().X().value();
  const double ty = result.points.back().Y().value();

  int prevClosest = 1;
  double currentSpeed = 0.0;  // robot starts at rest
  for (int tick = 0; tick < maxTicks; ++tick) {
    trajectory.emplace_back((x / nodeSizeMeters) * CELL_SIZE,
                            (y / nodeSizeMeters) * CELL_SIZE);

    if (std::hypot(tx - x, ty - y) < nodeSizeMeters * 0.15) {
      break;
    }

    // same closest point as controller
    int closest = prevClosest;
    double best = std::hypot(result.points[closest].X().value() - x,
                             result.points[closest].Y().value() - y);
    for (int i = prevClosest + 1; i < static_cast<int>(result.points.size());
         ++i) {
      double d = std::hypot(result.points[i].X().value() - x,
                            result.points[i].Y().value() - y);
      if (d < best) {
        best = d;
        closest = i;
      }
    }
    prevClosest = closest;

    double cx = result.points[closest].X().value();
    double cy = result.points[closest].Y().value();
    double error = std::hypot(cx - x, cy - y);

    if (error > errorThreshold) {
      // start the new profile from the current speed so that we don't slow down to 0 speed every time we replan
      result = CreateSpline(grid, toGridPoint(x, y, getNodeSize()), end,
                            nodeSizeMeters, 200);
      velocity_profile = CreateVelocityProfile(result, currentSpeed);
      prevClosest = 1;
      continue;
    }

    int vidx = std::min(closest, static_cast<int>(velocity_profile.size()) - 1);

    double vx = velocity_profile[vidx].first + kp * (cx - x) +
                (distr(gen) * k * currentSpeed);
    double vy = velocity_profile[vidx].second + kp * (cy - y) +
                (distr(gen) * k * currentSpeed);

    currentSpeed = std::hypot(vx, vy);

    x += vx * dt;
    y += vy * dt;
  }
  return trajectory;
}

auto drawObstacles(cv::Mat& canvas,
                   const std::vector<std::vector<Node>>& grid) {
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      if (grid[y][x].obstacle) {
        cv::rectangle(
            canvas,
            cv::Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            cv::Scalar(0, 0, 0), cv::FILLED);
      }
    }
  }
}

auto drawPath(cv::Mat& canvas, std::vector<std::pair<double, double>> path,
              const cv::Scalar& color) {

  for (size_t i = 1; i < path.size(); ++i) {
    cv::line(canvas, cv::Point(path[i - 1].first, path[i - 1].second),
             cv::Point(path[i].first, path[i].second), color, 2);
  }
}

}  // namespace pathing

auto main() -> int {
  pathing::getGrid();
  cv::Mat canvas(pathing::GRID_H * pathing::CELL_SIZE,
                 pathing::GRID_W * pathing::CELL_SIZE, CV_8UC3);
  canvas.setTo(cv::Scalar(255, 255, 255));

  pathing::Point start = {.x = 10, .y = 6};
  pathing::Point end = {.x = 46, .y = 12};

  auto expectedPath =
      pathing::generateExpectedPath(start, end, pathing::getNodeSize());
  auto noisyPath =
      pathing::simulateRobotPath(start, end, pathing::getNodeSize());

  pathing::drawObstacles(canvas, pathing::getGrid());
  // pathing::drawPath(canvas, expectedPath, cv::Scalar(0, 0, 255));
  pathing::drawPath(canvas, noisyPath, cv::Scalar(255, 0, 0));

  cv::imwrite("/tmp/xlo.png", canvas);
  return 0;
}
