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

auto generateExpectedPath(Point start, Point end, double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  const auto& grid = getGrid();
  SplineResult result = CreateSpline(grid, start, end, nodeSizeMeters, 20);

  std::vector<std::pair<double, double>> positions;
  for (const auto& pt : result.points) {
    double px = (pt.X().value() / nodeSizeMeters) * CELL_SIZE;
    double py = (pt.Y().value() / nodeSizeMeters) * CELL_SIZE;
    positions.emplace_back(px, py);
  }
  return positions;
}

auto generateNoisyPath(Point start, Point end, double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  const auto& grid = getGrid();
  SplineResult result = CreateSpline(grid, start, end, nodeSizeMeters, 20);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distr(-1, 1);

  std::vector<std::pair<double, double>> positions;
  for (const auto& pt : result.points) {
    double px = (pt.X().value() / nodeSizeMeters) * CELL_SIZE + distr(gen);
    double py = (pt.Y().value() / nodeSizeMeters) * CELL_SIZE + distr(gen);
    positions.emplace_back(px, py);
  }
  return positions;
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
  pathing::Point end = {.x = 44, .y = 12};

  auto path = pathing::generateExpectedPath(start, end, pathing::getNodeSize());
  LOG(INFO) << "Path points: " << path.size();
  if (!path.empty()) {
    LOG(INFO) << "First point: " << path[0].first << ", " << path[0].second;
  }

  pathing::drawPath(canvas, path, cv::Scalar(0, 0, 255));

  cv::imwrite("/tmp/xlo.png", canvas);
  cv::namedWindow("Pathing Simulator", cv::WINDOW_AUTOSIZE);
  return 0;
}
