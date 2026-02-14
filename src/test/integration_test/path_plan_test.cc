#include <frc/DataLogManager.h>
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <units/angle.h>
#include <units/length.h>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <utility>
#include <vector>
#include "src/utils/pch.h"

class Node {
 public:
  int x = 0;
  int y = 0;
  bool walkable = true;
  Node* parent = nullptr;
  double g = INFINITY;
  cv::Scalar color = {200, 200, 200};
};

std::ifstream file("/root/bos/constants/navgrid.json");
nlohmann::json data = nlohmann::json::parse(file);

const int GRID_W = data["grid"][0].size();
const int GRID_H = data["grid"].size();
double nodeSizeMeters = data["nodeSizeMeters"];
const int CELL_SIZE = 20;
std::vector<std::vector<Node>> grid(GRID_H, std::vector<Node>(GRID_W));

auto nodeRect(const Node& n) -> cv::Rect {
  return {n.x * CELL_SIZE, n.y * CELL_SIZE, CELL_SIZE, CELL_SIZE};
}

void drawGrid(cv::Mat& canvas, const std::vector<std::vector<Node>>& grid) {
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      const Node& n = grid[y][x];
      cv::rectangle(canvas, nodeRect(n), n.color, cv::FILLED);
      cv::rectangle(canvas, nodeRect(n), {50, 50, 50}, 1);
    }
  }
}

void makeObstacle(std::vector<std::vector<Node>>& grid, int row, int col) {
  grid[row][col].walkable = false;
  grid[row][col].color = {0, 0, 0};
}

void resetGrid(std::vector<std::vector<Node>>& grid) {
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      if (grid[y][x].walkable) {
        grid[y][x].color = {200, 200, 200};
      }
      grid[y][x].parent = nullptr;
      grid[y][x].g = INFINITY;
    }
  }
}

auto getRandomWalkableNode(const std::vector<std::vector<Node>>& grid,
                           std::mt19937& rng) -> Node* {
  std::vector<Node*> walkableNodes;
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      if (grid[y][x].walkable) {
        walkableNodes.push_back(const_cast<Node*>(&grid[y][x]));
      }
    }
  }

  if (walkableNodes.empty())
    return nullptr;

  std::uniform_int_distribution<> dist(0, walkableNodes.size() - 1);
  return walkableNodes[dist(rng)];
}

auto reconstructPath(Node* target) -> std::vector<Node*> {
  std::vector<Node*> path;
  Node* current = target;
  while (current != nullptr) {
    path.push_back(current);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void BFS(std::vector<std::vector<Node>>& grid, Node* start, Node* target) {
  std::priority_queue<std::pair<double, Node*>,
                      std::vector<std::pair<double, Node*>>, std::greater<>>
      openList;
  start->g = 0.0f;
  start->color = {0, 255, 0};
  target->color = {255, 0, 0};
  openList.push(std::make_pair(start->g, start));

  std::vector<std::pair<int, int>> dirs = {{0, -1},  {0, 1},  {-1, 0}, {1, 0},
                                           {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

  while (!openList.empty()) {
    std::pair<double, Node*> currentPair = openList.top();
    openList.pop();
    Node* current = currentPair.second;
    if (current == target)
      return;

    for (const std::pair<int, int>& d : dirs) {
      int nx = current->x + d.first;
      int ny = current->y + d.second;
      if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
        continue;
      Node* neighbor = &grid[ny][nx];
      if (!neighbor->walkable)
        continue;
      auto dx = static_cast<double>(d.first);
      auto dy = static_cast<double>(d.second);
      double cost = std::sqrt(dx * dx + dy * dy);
      double tentativeG = current->g + cost;
      if (tentativeG < neighbor->g) {
        neighbor->g = tentativeG;
        neighbor->parent = current;
        openList.push(std::make_pair(neighbor->g, neighbor));
      }
    }
  }
}

auto constructLinePath(cv::Mat& canvas, std::vector<Node*> path)
    -> std::vector<std::pair<int, int>> {
  std::vector<std::pair<int, int>> controlPoints;
  for (size_t i = 0; i < path.size(); ++i) {
    int px = path[i]->x * CELL_SIZE + (CELL_SIZE / 2);
    int py = path[i]->y * CELL_SIZE + (CELL_SIZE / 2);
    controlPoints.emplace_back(px, py);

    if (i > 0) {
      int prevX = path[i - 1]->x * CELL_SIZE + (CELL_SIZE / 2);
      int prevY = path[i - 1]->y * CELL_SIZE + (CELL_SIZE / 2);
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
      if (knots[i] <= t && t < knots[i + 1])
        return 1.0;

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

void drawSpline(cv::Mat& canvas, const std::vector<std::pair<int, int>>& points,
                std::vector<double>& knots, int p) {
  int res = 200;
  std::pair<double, double> prevPoint = {static_cast<double>(points[0].first),
                                         static_cast<double>(points[0].second)};

  for (int i = 1; i < res; ++i) {
    double t = (double)i / res;
    std::pair<double, double> currPoint = getSplinePoint(t, points, knots, p);

    cv::line(canvas, cv::Point(prevPoint.first, prevPoint.second),
             cv::Point(currPoint.first, currPoint.second),
             cv::Scalar(255, 0, 0), 2);

    prevPoint = currPoint;
  }
}

auto main() -> int {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("path_planning");
  inst.SetServerTeam(971);
  frc::DataLogManager::Start("/bos/logs/");

  std::vector<std::vector<int>> navgrid = data["grid"];
  std::random_device rd;
  std::mt19937 rng(rd());

  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      grid[y][x].x = x;
      grid[y][x].y = y;
      if (static_cast<bool>(navgrid[y][x])) {
        makeObstacle(grid, y, x);
      }
    }
  }

  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("PathPlanning");
  nt::StructArrayTopic<frc::Pose2d> trajectory_topic =
      table->GetStructArrayTopic<frc::Pose2d>("Trajectory");
  nt::StructArrayPublisher<frc::Pose2d> trajectory_publisher =
      trajectory_topic.Publish();

  while (true) {
    resetGrid(grid);

    Node* start = getRandomWalkableNode(grid, rng);
    Node* target = getRandomWalkableNode(grid, rng);

    if (!start || !target || start == target) {
      std::cout << "Failed to find valid start/target nodes" << std::endl;
      continue;
    }

    std::cout << "Start: (" << start->x << ", " << start->y << ") Target: ("
              << target->x << ", " << target->y << ")" << std::endl;

    BFS(grid, start, target);

    std::vector<Node*> path = reconstructPath(target);
    if (path.size() <= 1) {
      std::cout << "No path found, trying again..." << std::endl;
      continue;
    }

    for (Node* n : path) {
      if (n != start && n != target) {
        n->color = {0, 0, 255};
      }
    }

    cv::Mat canvas(GRID_H * CELL_SIZE, GRID_W * CELL_SIZE, CV_8UC3,
                   {255, 255, 255});
    drawGrid(canvas, grid);

    std::vector<std::pair<int, int>> controlPoints =
        constructLinePath(canvas, path);
    std::vector<double> knots =
        clampedUniformKnotVector(controlPoints.size(), 3.7);
    drawSpline(canvas, controlPoints, knots, 3);

    std::vector<frc::Pose2d> trajectory;
    int resolution = 200;
    for (int i = 0; i < resolution; ++i) {
      double t = (double)i / resolution;
      std::pair<double, double> point =
          getSplinePoint(t, controlPoints, knots, static_cast<int>(3.7));
      double px = point.first / CELL_SIZE;
      double py = point.second / CELL_SIZE;
      trajectory.push_back(frc::Pose2d(units::meter_t{px * nodeSizeMeters},
                                       units::meter_t{py * nodeSizeMeters},
                                       units::radian_t{0.0}));
    }
    trajectory_publisher.Set(trajectory);
    inst.Flush();

    cv::imshow("Path Planning", canvas);

    int key = cv::waitKey(0);
    if (key == 27) {
      break;
    }
  }

  return 0;
}
