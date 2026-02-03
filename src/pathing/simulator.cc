#include <frc/geometry/Pose2d.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <system_error>
#include <vector>
#include "src/pathing/pathing.h"

auto main() -> int {
  wpi::log::DataLogBackgroundWriter log{"/root/bos/logs", "sim.wpilog"};

  wpi::log::StructLogEntry<frc::Pose2d> poseLog(log, "/sim/Pose2d");
  wpi::log::DoubleLogEntry accelXLog(log, "/sim/AccelX");
  wpi::log::DoubleLogEntry accelYLog(log, "/sim/AccelY");
  wpi::log::DoubleLogEntry accelMagLog(log, "/sim/AccelMagnitude");
  wpi::log::DoubleLogEntry velXLog(log, "/sim/VelX");
  wpi::log::DoubleLogEntry velYLog(log, "/sim/VelY");

  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    return 1;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_W = data["grid"][0].size();
  const int GRID_H = data["grid"].size();
  double nodeSizeMeters = data["nodeSizeMeters"];

  std::vector<std::vector<bool>> gridData(GRID_H, std::vector<bool>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      gridData[y][x] = data["grid"][y][x];
    }
  }

  cv::Mat grid = initializeGrid(gridData);

  auto poses = createSpline(grid, 10, 5, 45, 22, nodeSizeMeters);
  if (poses.empty()) {
    return 1;
  }

  constexpr int64_t kDtUs = 100'000;
  constexpr double kDtSec = kDtUs / 1'000'000.0;
  constexpr double kMaxAccel = 3.0;
  constexpr double kMaxDecel = 3.0;
  constexpr double kMaxModuleSpeed = 5.0;
  int64_t t = 0;

  std::vector<double> pathDist(poses.size(), 0.0);
  for (size_t i = 1; i < poses.size(); ++i) {
    double dx = poses[i].X().value() - poses[i - 1].X().value();
    double dy = poses[i].Y().value() - poses[i - 1].Y().value();
    pathDist[i] = pathDist[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  double totalDist = pathDist.back();

  std::vector<double> targetSpeed(poses.size());

  for (size_t i = 0; i < poses.size(); ++i) {
    double distFromStart = pathDist[i];
    double distToEnd = totalDist - pathDist[i];

    double accelLimitedSpeed = std::sqrt(2.0 * kMaxAccel * distFromStart);
    double decelLimitedSpeed = std::sqrt(2.0 * kMaxDecel * distToEnd);

    double maxVelThisPose = kMaxModuleSpeed;
    if (i > 0) {
      double dx = poses[i].X().value() - poses[i - 1].X().value();
      double dy = poses[i].Y().value() - poses[i - 1].Y().value();
      double vx = dx / kDtSec;
      double vy = dy / kDtSec;
      double maxComponentSpeed = std::max(std::abs(vx), std::abs(vy));
      if (maxComponentSpeed > 0.001) {
        double speedRatio = std::sqrt(vx * vx + vy * vy) / maxComponentSpeed;
        maxVelThisPose = kMaxModuleSpeed / speedRatio;
      }
    }

    targetSpeed[i] =
        std::min({accelLimitedSpeed, decelLimitedSpeed, maxVelThisPose});
  }

  double currentVx = 0.0;
  double currentVy = 0.0;
  double currentX = poses[0].X().value();
  double currentY = poses[0].Y().value();
  double currentSpeed = 0.0;

  constexpr double kLookaheadDist = 0.5;  // meters
  constexpr double kGoalTolerance = 0.1;  // meters
  constexpr int kFlushInterval = 1;       // 10 iterations

  size_t currentWaypoint = 0;
  int iterationCount = 0;

  while (true) {
    double robotX = currentX;
    double robotY = currentY;

    frc::Pose2d actualPose{
        units::meter_t{robotX}, units::meter_t{robotY},
        poses[std::min(currentWaypoint, poses.size() - 1)].Rotation()};
    poseLog.Append(actualPose, t);

    size_t targetIdx = currentWaypoint;
    double minDist = INFINITY;

    for (size_t i = currentWaypoint; i < poses.size(); ++i) {
      double dx = poses[i].X().value() - robotX;
      double dy = poses[i].Y().value() - robotY;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist >= kLookaheadDist) {
        targetIdx = i;
        break;
      }

      if (dist < minDist) {
        minDist = dist;
        currentWaypoint = i;
      }
    }

    // Check if we reached the goal
    double goalDx = poses.back().X().value() - robotX;
    double goalDy = poses.back().Y().value() - robotY;
    double goalDist = std::sqrt(goalDx * goalDx + goalDy * goalDy);

    if (goalDist < kGoalTolerance) {

      accelXLog.Append(0.0, t);
      accelYLog.Append(0.0, t);
      accelMagLog.Append(0.0, t);
      velXLog.Append(0.0, t);
      velYLog.Append(0.0, t);
      break;
    }

    if (targetIdx >= poses.size() - 1) {
      targetIdx = poses.size() - 1;
    }

    double targetX = poses[targetIdx].X().value();
    double targetY = poses[targetIdx].Y().value();
    double dx = targetX - robotX;
    double dy = targetY - robotY;
    double distToTarget = std::sqrt(dx * dx + dy * dy);

    double dirX = distToTarget > 0.001 ? dx / distToTarget : 0.0;
    double dirY = distToTarget > 0.001 ? dy / distToTarget : 0.0;

    double desiredSpeed = targetSpeed[targetIdx];

    double dvMag = desiredSpeed - currentSpeed;
    double accelMag = 0.0;

    if (dvMag > 0) {
      accelMag = std::min(dvMag / kDtSec, kMaxAccel);
    } else {
      accelMag = std::max(dvMag / kDtSec, -kMaxDecel);
    }

    currentSpeed += accelMag * kDtSec;
    currentSpeed = std::max(0.0, currentSpeed);

    double newVx = dirX * currentSpeed;
    double newVy = dirY * currentSpeed;

    double ax = (newVx - currentVx) / kDtSec;
    double ay = (newVy - currentVy) / kDtSec;

    currentVx = newVx;
    currentVy = newVy;

    currentX += currentVx * kDtSec;
    currentY += currentVy * kDtSec;

    double accelMagTotal = std::sqrt(ax * ax + ay * ay);

    accelXLog.Append(ax, t);
    accelYLog.Append(ay, t);
    accelMagLog.Append(accelMagTotal, t);
    velXLog.Append(currentVx, t);
    velYLog.Append(currentVy, t);

    t += kDtUs;

    if (++iterationCount % kFlushInterval == 0) {
      log.Flush();
    }
  }

  log.Flush();
  return 0;
}