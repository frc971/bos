#include "src/pathing/controller.h"
#include <frc/Timer.h>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>
#include "src/pathing/pathing.h"
#include "src/pathing/velocity_sender.h"

namespace pathing {

Controller::Controller() : instance_(nt::NetworkTableInstance::GetDefault()) {

  auto table = instance_.GetTable("Pathing");

  current_pose_sub_ =
      table->GetStructTopic<frc::Pose2d>("CurrentPose").Subscribe({});

  target_pose_sub_ =
      table->GetStructTopic<frc::Pose2d>("TargetPose").Subscribe({});
}

void Controller::Send() {

  VelocitySender sender;

  std::ifstream file("/root/bos/constants/navgrid.json");
  nlohmann::json data = nlohmann::json::parse(file);

  int W = data["grid"][0].size();
  int H = data["grid"].size();
  double nodeSize = data["nodeSizeMeters"];

  std::vector<std::vector<bool>> gridData(H, std::vector<bool>(W));

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      gridData[y][x] = data["grid"][y][x];

  cv::Mat grid = initializeGrid(gridData);

  frc::Pose2d start = current_pose_sub_.Get();
  frc::Pose2d target = target_pose_sub_.Get();

  int sx = start.X().value() / nodeSize;
  int sy = start.Y().value() / nodeSize;
  int tx = target.X().value() / nodeSize;
  int ty = target.Y().value() / nodeSize;

  auto poses = createSpline(grid, sx, sy, tx, ty, nodeSize);

  if (poses.empty()) {
    sender.Send(0, 0);
    return;
  }

  std::vector<double> dist(poses.size(), 0);

  for (size_t i = 1; i < poses.size(); i++) {
    double dx = poses[i].X().value() - poses[i - 1].X().value();
    double dy = poses[i].Y().value() - poses[i - 1].Y().value();
    dist[i] = dist[i - 1] + std::hypot(dx, dy);
  }

  double total = dist.back();

  constexpr double maxV = 5.0;
  constexpr double maxA = 3.0;
  constexpr double lookahead = 0.5;
  constexpr double tol = 0.1;

  std::vector<double> speed(poses.size());

  for (size_t i = 0; i < poses.size(); i++) {
    double a = std::sqrt(2 * maxA * dist[i]);
    double d = std::sqrt(2 * maxA * (total - dist[i]));
    speed[i] = std::min({a, d, maxV});
  }

  size_t idx = 0;

  while (true) {

    frc::Pose2d pose = current_pose_sub_.Get();
    double rx = pose.X().value();
    double ry = pose.Y().value();

    double goalDist = std::hypot(poses.back().X().value() - rx,
                                 poses.back().Y().value() - ry);

    if (goalDist < tol) {
      sender.Send(0, 0);
      break;
    }

    for (size_t i = idx; i < poses.size(); i++) {
      double d =
          std::hypot(poses[i].X().value() - rx, poses[i].Y().value() - ry);
      if (d >= lookahead) {
        idx = i;
        break;
      }
    }

    if (idx >= poses.size())
      idx = poses.size() - 1;

    double txp = poses[idx].X().value();
    double typ = poses[idx].Y().value();

    double dx = txp - rx;
    double dy = typ - ry;
    double mag = std::hypot(dx, dy);

    if (mag < 1e-6) {
      sender.Send(0, 0);
      continue;
    }

    double vx = dx / mag * speed[idx];
    double vy = dy / mag * speed[idx];

    sender.Send(vx, vy);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}  // namespace pathing