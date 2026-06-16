#include <frc/geometry/Pose2d.h>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <utility>
#include <vector>
#include "src/pathing/pathfinding.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

namespace pathing {

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

    const int GRID_H = data["grid"].size();
    const int GRID_W = data["grid"][0].size();

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

auto generateExpectedPath(Point start, Point end, double nodeSizeMeters)
    -> SplineResult {
  SplineResult result;
  const auto& grid = getGrid();
  result = CreateSpline(grid, start, end, nodeSizeMeters, 20);
  return result;
}

auto generateNoisyPath(Point start, Point end, double nodeSizeMeters)
    -> SplineResult {
  SplineResult result;
  const auto& grid = getGrid();
  result = CreateSpline(grid, start, end, nodeSizeMeters, 20);

  std::random_device rd;

  std::mt19937 gen(rd());

  std::uniform_real_distribution<> distr(-1, 1);

  for (auto& pt : result.points) {
    double noiseX = distr(gen);
    double noiseY = distr(gen);
    pt = frc::Pose2d(pt.X() + units::meter_t{noiseX},
                     pt.Y() + units::meter_t{noiseY}, pt.Rotation());
  }

  return result;

}  // namespace pathing