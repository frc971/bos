#include "spline_follower.h"

#include <algorithm>
#include <cmath>
#include "src/utils/log.h"

namespace pathing {

auto FollowSpline(const frc::Pose2d& current_pose,
                  const std::vector<frc::Pose2d>& spline_points,
                  int lookahead_offset,
                  double speed,
                  bool verbose) -> FollowResult {
  int closest_idx = 0;

  // TODO: this can be optimizeed by only searching from the previous closest index instead of the entire spline
  frc::Translation2d first2d(spline_points[0].X(), spline_points[0].Y());
  double best_dist = current_pose.Translation().Distance(first2d).value();
  for (int i = 1; i < (int)spline_points.size(); ++i) {
    frc::Translation2d t2d(spline_points[i].X(), spline_points[i].Y());
    double d = current_pose.Translation().Distance(t2d).value();
    if (d < best_dist) {
      best_dist = d;
      closest_idx = i;
    }
    if (verbose) {
      LOG(INFO) << "d: " << d << " i: " << i;
    }
  }

  if (verbose) {
    LOG(INFO) << "Closeset idx: " << closest_idx
              << " Spline size: " << spline_points.size();
  }

  int lookahead_idx =
      std::min(closest_idx + lookahead_offset, (int)spline_points.size() - 1);

  frc::Pose2d lookahead = spline_points[lookahead_idx];

  if (verbose) {
    LOG(INFO) << "current " << current_pose.X().value() << " "
              << current_pose.Y().value();
  }

  double dx = lookahead.X().value() - current_pose.X().value();
  double dy = lookahead.Y().value() - current_pose.Y().value();

  if (verbose) {
    LOG(INFO) << "dx " << dx << " vy " << dy;
    LOG(INFO) << "looakhead " << lookahead.X().value() << " "
              << lookahead.Y().value();
  }

  double dist = std::hypot(dx, dy);
  if (verbose) {
    LOG(INFO) << "dist " << dist;
  }

  if (dist < 1e-6) {
    return {0.0, 0.0, lookahead};
  }

  double vx = dx * speed;
  double vy = dy * speed;

  if (verbose) {
    LOG(INFO) << "set vx " << vx << " vy " << vy;
  }

  return {vx, vy, lookahead};
}

}  // namespace pathing
