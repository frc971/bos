#include "src/pathing/controller.h"
#include "src/pathing/pathing.h"
#include "src/pathing/velocity_sender.h"
#include <frc/Timer.h>
#include <chrono>
#include <thread>

namespace pathing {

Controller::Controller()
    : instance_(nt::NetworkTableInstance::GetDefault()) {
  std::shared_ptr<nt::NetworkTable> table = instance_.GetTable("Pathing");
  nt::StructTopic<frc::Pose2d> current_pose_topic =
      table->GetStructTopic<frc::Pose2d>("CurrentPose");
  current_pose_sub_ = current_pose_topic.Subscribe({});
  
  nt::StructTopic<frc::Pose2d> target_pose_topic =
      table->GetStructTopic<frc::Pose2d>("TargetPose");
  target_pose_sub_ = target_pose_topic.Subscribe({});
}

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

    frc::Pose2d startPose = current_pose_sub_.Get();
    frc::Pose2d targetPose = target_pose_sub_.Get();
    
    int startX = static_cast<int>(startPose.X().value() / nodeSizeMeters);
    int startY = static_cast<int>(startPose.Y().value() / nodeSizeMeters);
    int targetX = static_cast<int>(targetPose.X().value() / nodeSizeMeters);
    int targetY = static_cast<int>(targetPose.Y().value() / nodeSizeMeters);

    auto poses = createSpline(grid, startX, startY, targetX, targetY, nodeSizeMeters);

    constexpr int64_t kDtUs = 100'000;
    constexpr double kDtSec = kDtUs / 1'000'000.0;
    constexpr double kMaxAccel = 3.0;
    constexpr double kMaxDecel = 3.0;
    constexpr double kMaxModuleSpeed = 5.0;
    
    frc::Timer timer;
    timer.Start();

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
    double currentSpeed = 0.0;
    
    constexpr double kLookaheadDist = 0.5;  // meters
    constexpr double kGoalTolerance = 0.1;  // meters
    
    size_t currentWaypoint = 0;
    bool pathComplete = false;

    while (!pathComplete) {
      frc::Pose2d actualPose = current_pose_sub_.Get();
      double robotX = actualPose.X().value();
      double robotY = actualPose.Y().value();
      
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
      
      double goalDx = poses.back().X().value() - robotX;
      double goalDy = poses.back().Y().value() - robotY;
      double goalDist = std::sqrt(goalDx * goalDx + goalDy * goalDy);
      
      if (goalDist < kGoalTolerance) {
        pathComplete = true;
        velocity_sender.Send(0.0, 0.0);
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
      
      // TODO: find the max acceleration of the robot in order to calibrate this
      
      currentSpeed += accelMag * kDtSec;
      currentSpeed = std::max(0.0, currentSpeed);
      
      double newVx = dirX * currentSpeed;
      double newVy = dirY * currentSpeed;
      
      double ax = (newVx - currentVx) / kDtSec;
      double ay = (newVy - currentVy) / kDtSec;
      
      currentVx = newVx;
      currentVy = newVy;
      
      velocity_sender.Send(ax, ay);
      
      std::this_thread::sleep_for(std::chrono::microseconds(kDtUs));
    }

  }
}  // namespace pathing
  
