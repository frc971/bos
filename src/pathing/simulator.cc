#include <absl/log/log.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/DataLogWriter.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <system_error>
#include <vector>
#include "src/pathing/pathing.h"

constexpr uint CELL_SIZE = 20;

using namespace pathing;

auto main() -> int {
  std::string logDir = "./logs";
  std::filesystem::create_directories(logDir);
  std::filesystem::remove(logDir + "/sim.wpilog");
  std::error_code ec;
  auto log =
      std::make_unique<wpi::log::DataLogWriter>("localization_log.wpilog", ec);
  if (ec) {
    std::cerr << "Failed to open log: " << ec.message() << std::endl;
    return 0;
  }

  wpi::log::StructLogEntry<frc::Pose2d> poseLog(*log, "/sim/Pose");
  wpi::log::DoubleLogEntry accelXLog(*log, "/sim/AccelX");
  wpi::log::DoubleLogEntry accelYLog(*log, "/sim/AccelY");
  wpi::log::DoubleLogEntry accelMagLog(*log, "/sim/AccelMagnitude");
  wpi::log::DoubleLogEntry velXLog(*log, "/sim/VelX");
  wpi::log::DoubleLogEntry velYLog(*log, "/sim/VelY");

  std::ifstream file("/bos/constants/navgrid.json");
  if (!file.is_open()) {
    LOG(FATAL) << "Unable to find navgrid";
    return 1;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_W = data["grid"][0].size();
  const int GRID_H = data["grid"].size();

  std::vector<std::vector<bool>> gridData(GRID_H, std::vector<bool>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      gridData[y][x] = data["grid"][y][x];
    }
  }

  cv::Mat grid = InitializeGrid(gridData);

  auto poses = CreateSpline(grid, 10, 5, 45, 22);
  if (poses.empty()) {
    LOG(FATAL) << "Spline is empty";
    return 1;
  }

  constexpr double kDtSec = 0.02;
  constexpr double kMaxAccel = 3.0;
  constexpr double kMaxDecel = 3.0;
  constexpr double kMaxModuleSpeed = 5.0;
  double t = 0;

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

  for (size_t i = 0; i < poses.size(); ++i) {
    frc::Translation2d actualPose{units::meter_t{currentX},
                                  units::meter_t{currentY}};
    std::cout << "Appending: x: " << currentX << ", y: " << currentY
              << std::endl;
    poseLog.Append(frc::Pose2d{actualPose, frc::Rotation2d{units::radian_t{0}}},
                   t);
    std::cout << "Appended" << std::endl;

    double desiredSpeed = targetSpeed[i];

    double dvMag = desiredSpeed - currentSpeed;
    double accelMag = 0.0;

    if (dvMag > 0) {
      accelMag = std::min(dvMag / kDtSec, kMaxAccel);
    } else {
      accelMag = std::max(dvMag / kDtSec, -kMaxDecel);
    }

    currentSpeed += accelMag * kDtSec;
    currentSpeed = std::max(0.0, currentSpeed);

    if (i > 0) {
      double dx = poses[i].X().value() - poses[i - 1].X().value();
      double dy = poses[i].Y().value() - poses[i - 1].Y().value();
      double segDist = std::sqrt(dx * dx + dy * dy);

      double dirX = segDist > 0.001 ? dx / segDist : 0.0;
      double dirY = segDist > 0.001 ? dy / segDist : 0.0;

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
    }

    t += kDtSec;
  }

  std::cout << "Stopping this log" << std::endl;
  log->Stop();
  return 0;
}

auto ConstructLinePath(cv::Mat& canvas, std::vector<cv::Point2i> path)
    -> std::vector<cv::Point2i> {
  std::vector<cv::Point2i> controlPoints;
  for (size_t i = 0; i < path.size(); ++i) {
    int px = path[i].x * CELL_SIZE + (CELL_SIZE / 2);
    int py = path[i].y * CELL_SIZE + (CELL_SIZE / 2);

    controlPoints.emplace_back(px, py);

    if (i > 0) {
      int prevX = path[i - 1].x * CELL_SIZE + (CELL_SIZE / 2);
      int prevY = path[i - 1].y * CELL_SIZE + (CELL_SIZE / 2);
      cv::line(canvas, cv::Point(prevX, prevY), cv::Point(px, py), {0, 0, 0},
               2);
    }
  }
  return controlPoints;
}