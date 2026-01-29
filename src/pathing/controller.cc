#include "src/pathing/controller.h"
#include "src/pathing/pathing.h"
#include "src/pathing/velocity_sender.h"

namespace pathing {
  void Controller::Send() { 
    VelocitySender velocity_sender;

    std::ifstream file("/root/bos/constants/navgrid.json");

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

    constexpr int64_t kDtUs = 20'000;
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

    for (size_t i = 0; i < poses.size(); ++i) {
      // TODO: get this from the java networktables agent side
        frc::Pose2d actualPose{units::meter_t{currentX}, units::meter_t{currentY},
                             poses[i].Rotation()};

      double desiredSpeed = targetSpeed[i];

      double dvMag = desiredSpeed - currentSpeed;
      double accelMag = 0.0;

      if (dvMag > 0) {
        accelMag = std::min(dvMag / kDtSec, kMaxAccel);
      } else {
        accelMag = std::max(dvMag / kDtSec, -kMaxDecel);
      }

      // TODO: find the max acceleration of the robot in order to calibrate this

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

        // TODO: make this compatible with the java agent and figure out how it will all work
        // velocity_sender.Send(ax, ay, static_cast<double>(t) / 1'000'000.0);

        std::cout << ax;
        std::cout << ", " << ay << std::endl;

      }

      t += kDtUs;
    }

  }
}  // namespace pathing
  
