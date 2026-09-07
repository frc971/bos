#include <frc/geometry/Pose2d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <units/length.h>
#include <sys/stat.h>
#include <cmath>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include "src/pathing/path_follower.h"
#include "src/pathing/pathfinding.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"
#include "wpi/DataLogWriter.h"

namespace pathing {

int CELL_SIZE = 20;

// Runs BFS leg-by-leg through `waypoints` (so the path is forced through
// each intermediate point), but aggregates every leg's raw BFS nodes into
// one control-point list and fits a single spline across all of them —
// rather than fitting a separate spline per leg — so the curve is
// continuous through the zig-zag instead of having a seam at each waypoint.
auto buildAggregatedSpline(const std::vector<std::vector<Node>>& grid,
                           const std::vector<Point>& waypoints,
                           double nodeSizeMeters, int samples)
    -> SplineResult {
  std::vector<std::pair<double, double>> control_points;

  for (size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    std::vector<std::vector<Node>> gridCopy = grid;
    std::vector<Node> legPath = BFS(gridCopy, waypoints[leg], waypoints[leg + 1]);
    if (legPath.empty()) {
      LOG(INFO) << "BFS returned no path for leg " << leg;
      return {};
    }
    // Skip the first node of every leg after the first: it's the same
    // point as the previous leg's last node, and we don't want it twice.
    size_t startIdx = leg == 0 ? 0 : 1;
    for (size_t i = startIdx; i < legPath.size(); ++i) {
      control_points.emplace_back(legPath[i].x * nodeSizeMeters,
                                  legPath[i].y * nodeSizeMeters);
    }
  }

  uint numControls = control_points.size();
  if (numControls < 4) {
    return {};
  }

  uint p = 6;
  if (numControls <= p) {
    p = numControls - 1;
  }

  std::vector<double> knots = KnotVector(numControls, p);

  std::vector<frc::Pose2d> spline_points;
  std::vector<double> spline_params;
  for (int t = 0; t <= samples; ++t) {
    double normalized_t = t / static_cast<double>(samples);
    // space the points farther apart at the ends of the spline, same
    // easing CreateSpline uses, so the robot doesn't jitter at start/end
    double t_real =
        normalized_t <= 0.5
            ? 0.5 * std::pow(2.0 * normalized_t, 0.75)
            : 1.0 - 0.5 * std::pow(2.0 * (1.0 - normalized_t), 0.75);
    auto [x, y] = EvaluatePosition(t_real, control_points, knots, p);
    spline_points.emplace_back(units::meter_t{x}, units::meter_t{y}, 0_rad);
    spline_params.emplace_back(t_real);
  }

  return {spline_points, control_points, {}, knots, spline_params, p};
}

struct FollowerSample {
  double x, y, vx, vy;
};

// Drives the real PathFollower (the same controller class RunController
// uses in src/pathing/controller.cc) through `waypoints`, exactly like
// production: a single PathFollower instance for the whole run, fed
// whatever the current target is each tick via update() — the same call
// controller.cc makes every loop iteration. As in production, the
// follower's own internal reset()-on-done is what triggers replanning for
// the next target; we don't recreate or reset it ourselves between legs.
// `noiseFraction` is the fraction of current speed added as random
// velocity noise each tick — 0 for a clean "expected" run, >0 (0.15) for
// the noisy simulated-robot run.
auto followLegs(const std::vector<std::vector<Node>>& grid,
                double nodeSizeMeters, const std::vector<Point>& waypoints,
                double noiseFraction) -> std::vector<FollowerSample> {
  if (waypoints.size() < 2) {
    return {};
  }

  const double dt = 0.02;      // 20 ms, matches controller.cc's loop period
  const int maxTotalTicks = 2000;  // batch-run safety cap only

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distr(-1.0, 1);

  double x = waypoints.front().x * nodeSizeMeters;
  double y = waypoints.front().y * nodeSizeMeters;
  double currentSpeed = 0.0;

  PathFollower follower(grid, nodeSizeMeters, 10.0, 0.4, 1000);

  std::vector<FollowerSample> trajectory;
  size_t leg = 0;
  for (int tick = 0; tick < maxTotalTicks && leg + 1 < waypoints.size();
       ++tick) {
    Point legEnd = waypoints[leg + 1];
    frc::Pose2d target_pose(units::meter_t{legEnd.x * nodeSizeMeters},
                            units::meter_t{legEnd.y * nodeSizeMeters},
                            frc::Rotation2d{});
    frc::Pose2d current_pose(units::meter_t{x}, units::meter_t{y},
                             frc::Rotation2d{});
    FollowerOutput out = follower.update(current_pose, target_pose);
    if (out.done) {
      ++leg;
      continue;
    }

    double vx = out.vx + (distr(gen) * noiseFraction * currentSpeed);
    double vy = out.vy + (distr(gen) * noiseFraction * currentSpeed);
    currentSpeed = std::hypot(vx, vy);

    trajectory.push_back({x, y, vx, vy});

    x += vx * dt;
    y += vy * dt;
  }
  return trajectory;
}

auto samplesToPixels(const std::vector<FollowerSample>& samples,
                     double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  std::vector<std::pair<double, double>> pixels;
  pixels.reserve(samples.size());
  for (const auto& s : samples) {
    pixels.emplace_back((s.x / nodeSizeMeters) * CELL_SIZE,
                        (s.y / nodeSizeMeters) * CELL_SIZE);
  }
  return pixels;
}

auto splinePointsToPixels(const SplineResult& spline, double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  std::vector<std::pair<double, double>> pixels;
  pixels.reserve(spline.points.size());
  for (const auto& pt : spline.points) {
    pixels.emplace_back((pt.X().value() / nodeSizeMeters) * CELL_SIZE,
                        (pt.Y().value() / nodeSizeMeters) * CELL_SIZE);
  }
  return pixels;
}

auto controlsToPixels(const std::vector<std::pair<double, double>>& controls,
                      double nodeSizeMeters)
    -> std::vector<std::pair<double, double>> {
  std::vector<std::pair<double, double>> pixels;
  pixels.reserve(controls.size());
  for (const auto& [mx, my] : controls) {
    pixels.emplace_back((mx / nodeSizeMeters) * CELL_SIZE,
                        (my / nodeSizeMeters) * CELL_SIZE);
  }
  return pixels;
}

auto drawObstacles(cv::Mat& canvas,
                   const std::vector<std::vector<Node>>& grid) {
  for (int y = 0; y < static_cast<int>(grid.size()); ++y) {
    for (int x = 0; x < static_cast<int>(grid[0].size()); ++x) {
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

auto drawWaypoint(cv::Mat& canvas, Point p, const cv::Scalar& color,
                  int index) {
  cv::rectangle(canvas,
                cv::Rect(p.x * CELL_SIZE, p.y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
                color, cv::FILLED);
  cv::putText(canvas, std::to_string(index),
              cv::Point(p.x * CELL_SIZE + 2, p.y * CELL_SIZE + CELL_SIZE - 4),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1,
              cv::LINE_AA);
}

// Picks a uniformly random in-bounds point, then snaps it to the nearest
// free cell using the same BFS the pathfinder relies on to seed real runs.
// Runs BFSFirstFreeCell on a throwaway copy of the grid: that function
// permanently marks cells `visited` as it searches, and since the real
// BFS() also gates on `visited`, calling it on the live grid would corrupt
// every pathfinding call after the first.
auto randomFreePoint(const std::vector<std::vector<Node>>& grid,
                     std::mt19937& gen) -> Point {
  std::uniform_int_distribution<uint> xd(
      0, static_cast<uint>(grid[0].size()) - 1);
  std::uniform_int_distribution<uint> yd(0,
                                         static_cast<uint>(grid.size()) - 1);
  Point raw = {.x = xd(gen), .y = yd(gen)};
  std::vector<std::vector<Node>> gridCopy = grid;
  Node free = BFSFirstFreeCell(gridCopy, raw);
  return {.x = free.x, .y = free.y};
}

// Writes one trial's data to a .wpilog: the aggregated waypoints, and the
// no-noise "expected" and noisy pursuit-follower runs (position + velocity
// per tick).
auto writeTrialLog(const std::string& path, const std::vector<Point>& waypoints,
                   const std::vector<FollowerSample>& expected,
                   const std::vector<FollowerSample>& noisy) -> void {
  std::error_code ec;
  wpi::log::DataLogWriter log(path, ec);
  if (ec) {
    LOG(INFO) << "Failed to open wpilog " << path << ": " << ec.message();
    return;
  }

  std::vector<double> flatWaypoints;
  flatWaypoints.reserve(waypoints.size() * 2);
  for (const auto& wp : waypoints) {
    flatWaypoints.push_back(wp.x);
    flatWaypoints.push_back(wp.y);
  }
  wpi::log::DoubleArrayLogEntry waypointsEntry(log, "waypoints");
  waypointsEntry.Append(flatWaypoints);

  wpi::log::DoubleLogEntry expectedX(log, "expected/x");
  wpi::log::DoubleLogEntry expectedY(log, "expected/y");
  wpi::log::DoubleLogEntry expectedVx(log, "expected/vx");
  wpi::log::DoubleLogEntry expectedVy(log, "expected/vy");
  wpi::log::StructLogEntry<frc::Pose2d> expectedPose(log, "expected/pose");
  int64_t timestamp = 0;
  for (const auto& s : expected) {
    expectedX.Append(s.x, timestamp);
    expectedY.Append(s.y, timestamp);
    expectedVx.Append(s.vx, timestamp);
    expectedVy.Append(s.vy, timestamp);
    expectedPose.Append(frc::Pose2d(units::meter_t{s.x}, units::meter_t{s.y},
                                    frc::Rotation2d{}),
                        timestamp);
    timestamp += 20000;  // 20 ms in microseconds
  }

  wpi::log::DoubleLogEntry noisyX(log, "noisy/x");
  wpi::log::DoubleLogEntry noisyY(log, "noisy/y");
  wpi::log::DoubleLogEntry noisyVx(log, "noisy/vx");
  wpi::log::DoubleLogEntry noisyVy(log, "noisy/vy");
  wpi::log::StructLogEntry<frc::Pose2d> noisyPose(log, "noisy/pose");
  timestamp = 0;
  for (const auto& s : noisy) {
    noisyX.Append(s.x, timestamp);
    noisyY.Append(s.y, timestamp);
    noisyVx.Append(s.vx, timestamp);
    noisyVy.Append(s.vy, timestamp);
    noisyPose.Append(frc::Pose2d(units::meter_t{s.x}, units::meter_t{s.y},
                                 frc::Rotation2d{}),
                     timestamp);
    timestamp += 20000;
  }

  log.Flush();
}

struct TrialResult {
  cv::Mat canvas;
  std::vector<Point> waypoints;
  std::vector<FollowerSample> expected;
  std::vector<FollowerSample> noisy;
};

// Runs one poly-line trial: `legs` random waypoints chained together,
// fit as a single continuous spline and followed end-to-end.
auto runTrial(std::vector<std::vector<Node>>& grid, double nodeSizeMeters,
              int legs, std::mt19937& gen) -> TrialResult {
  cv::Mat canvas(static_cast<int>(grid.size()) * CELL_SIZE,
                 static_cast<int>(grid[0].size()) * CELL_SIZE, CV_8UC3);
  canvas.setTo(cv::Scalar(255, 255, 255));
  drawObstacles(canvas, grid);

  std::vector<Point> waypoints;
  waypoints.push_back(randomFreePoint(grid, gen));
  for (int leg = 0; leg < legs; ++leg) {
    waypoints.push_back(randomFreePoint(grid, gen));
  }

  SplineResult spline =
      buildAggregatedSpline(grid, waypoints, nodeSizeMeters, 200);

  std::vector<FollowerSample> expected;
  std::vector<FollowerSample> noisy;
  if (!spline.points.empty()) {
    // grey: the raw BFS poly-line the aggregated spline was fit to
    drawPath(canvas, controlsToPixels(spline.controls, nodeSizeMeters),
            cv::Scalar(160, 160, 160));
    // red: the aggregated spline curve itself (the ideal reference path)
    drawPath(canvas, splinePointsToPixels(spline, nodeSizeMeters),
            cv::Scalar(0, 0, 255));

    // blue: the real PathFollower controller driving the robot, leg by
    // leg, with noise. `expected` is the same controller with no noise,
    // logged for comparison but not drawn (it tracks the red curve).
    expected = followLegs(grid, nodeSizeMeters, waypoints, /*noiseFraction=*/0.0);
    noisy = followLegs(grid, nodeSizeMeters, waypoints, /*noiseFraction=*/0.15);

    drawPath(canvas, samplesToPixels(noisy, nodeSizeMeters),
            cv::Scalar(255, 0, 0));
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    cv::Scalar color = cv::Scalar(0, 165, 255);  // orange = mid waypoint
    if (i == 0) {
      color = cv::Scalar(0, 255, 0);  // green = trial start
    } else if (i == waypoints.size() - 1) {
      color = cv::Scalar(0, 255, 255);  // yellow = final target
    }
    drawWaypoint(canvas, waypoints[i], color, static_cast<int>(i));
  }

  return {canvas, waypoints, expected, noisy};
}

}  // namespace pathing

auto main() -> int {
  const auto& navgrid = pathing::GetGrid("/root/bos/constants/navgrid.json");
  auto grid = navgrid.grid;
  const auto& nodeSizeMeters = navgrid.nodeSizeMeters;

  const std::string outDir = std::string(getenv("HOME")) + "/pathing-simulator/";
  mkdir(outDir.c_str(), 0755);

  std::random_device rd;
  std::mt19937 gen(rd());
  // Half the trials are single-leg (simple point-to-point runs), the other
  // half chain 2-6 random legs to stress-test zig-zag spline fitting.
  std::uniform_int_distribution<int> singleOrMulti(0, 1);
  std::uniform_int_distribution<int> legCountDist(2, 6);

  const int numTrials = 100;
  for (int trialNum = 1; trialNum <= numTrials; ++trialNum) {
    int legs = singleOrMulti(gen) == 0 ? 1 : legCountDist(gen);
    LOG(INFO) << "Trial " << trialNum << "/" << numTrials << ": " << legs
              << " leg(s)";

    pathing::TrialResult result =
        pathing::runTrial(grid, nodeSizeMeters, legs, gen);
    cv::imwrite(outDir + std::to_string(trialNum) + ".png", result.canvas);
    pathing::writeTrialLog(outDir + std::to_string(trialNum) + ".wpilog",
                          result.waypoints, result.expected, result.noisy);
  }
  return 0;
}
