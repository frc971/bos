#include <frc/geometry/Pose2d.h>
#include <units/length.h>
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
#include "src/pathing/path_follower.h"
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
  PathFollower follower(grid, nodeSizeMeters, 10.0, 0.4, 200);

  const double dt = 0.02;     // 20 ms
  const double k = 0.15;      // noise fraction
  const int maxTicks = 2000;  // don't go infinite loop

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distr(-1.0, 1);

  double x = start.x * nodeSizeMeters;
  double y = start.y * nodeSizeMeters;

  frc::Pose2d target_pose(units::meter_t{end.x * nodeSizeMeters},
                          units::meter_t{end.y * nodeSizeMeters},
                          frc::Rotation2d{});

  std::vector<std::pair<double, double>> trajectory;
  double currentSpeed = 0.0;  // robot starts at rest
  for (int tick = 0; tick < maxTicks; ++tick) {
    trajectory.emplace_back((x / nodeSizeMeters) * CELL_SIZE,
                            (y / nodeSizeMeters) * CELL_SIZE);

    frc::Pose2d current_pose(units::meter_t{x}, units::meter_t{y},
                             frc::Rotation2d{});
    FollowerOutput out = follower.update(current_pose, target_pose);
    if (out.done) {
      break;
    }

    double vx = out.vx + (distr(gen) * k * currentSpeed);
    double vy = out.vy + (distr(gen) * k * currentSpeed);
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
  pathing::drawPath(canvas, expectedPath, cv::Scalar(0, 0, 255));
  pathing::drawPath(canvas, noisyPath, cv::Scalar(255, 0, 0));

  cv::imwrite("/tmp/xlo.png", canvas);
  return 0;
}
