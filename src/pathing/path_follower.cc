#include "path_follower.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include "src/utils/pch.h"
#include "velocity_profile.h"

namespace pathing {

PathFollower::PathFollower(const std::vector<std::vector<Node>>& grid,
                           double nodeSizeMeters, double kp,
                           double errorThreshold, int sampleCount)
    : grid_(grid),
      nodeSizeMeters_(nodeSizeMeters),
      kp_(kp),
      errorThreshold_(errorThreshold),
      sampleCount_(sampleCount) {}

auto PathFollower::resetPath() -> void {
  spline_.reset();
  velocity_profile_.reset();
  prev_closest_idx_ = std::nullopt;
}

auto PathFollower::reset() -> void {
  resetPath();
  last_vx_ = 0.0;
  last_vy_ = 0.0;
}

auto PathFollower::plan(const frc::Pose2d& current_pose,
                        const frc::Pose2d& target_pose) -> void {
  int gw = static_cast<int>(grid_[0].size());
  int gh = static_cast<int>(grid_.size());

  Point start{.x = static_cast<uint>(std::clamp(
                  static_cast<int>(current_pose.X().value() / nodeSizeMeters_),
                  0, gw - 1)),
              .y = static_cast<uint>(std::clamp(
                  static_cast<int>(current_pose.Y().value() / nodeSizeMeters_),
                  0, gh - 1))};
  Point target{.x = static_cast<uint>(std::clamp(
                   static_cast<int>(target_pose.X().value() / nodeSizeMeters_),
                   0, gw - 1)),
               .y = static_cast<uint>(std::clamp(
                   static_cast<int>(target_pose.Y().value() / nodeSizeMeters_),
                   0, gh - 1))};

  spline_ = CreateSpline(grid_, start, target, nodeSizeMeters_, sampleCount_);
  velocity_profile_ =
      CreateVelocityProfile(*spline_, std::hypot(last_vx_, last_vy_));
  prev_closest_idx_ = std::nullopt;
}

auto PathFollower::update(const frc::Pose2d& current_pose,
                          const frc::Pose2d& target_pose) -> FollowerOutput {
  // we are done if we are in the radius of the target
  frc::Translation2d target2d(target_pose.X(), target_pose.Y());
  if (current_pose.Translation().Distance(target2d).value() <
      nodeSizeMeters_ * 0.15) {
    reset();
    return {.vx = 0.0, .vy = 0.0, .done = true};
  }

  // no spline means we need to plan one
  if (!spline_.has_value()) {
    plan(current_pose, target_pose);
  }
  if (!spline_.has_value() || spline_->points.empty() ||
      !velocity_profile_.has_value() || velocity_profile_->empty()) {
    return {.vx = 0.0, .vy = 0.0, .done = false};
  }

  const auto& points = spline_->points;
  const auto& profile = *velocity_profile_;

  // closest point, forward-only from the previous index
  // TODO: this is a bit convoluted, might be worth it to fix later
  int start_idx = static_cast<int>(prev_closest_idx_.value_or(0));
  int closest_idx = start_idx;
  frc::Translation2d start2d(points[start_idx].X(), points[start_idx].Y());
  double best_dist = current_pose.Translation().Distance(start2d).value();
  for (int i = start_idx + 1; i < static_cast<int>(points.size()); ++i) {
    frc::Translation2d p2d(points[i].X(), points[i].Y());
    double d = current_pose.Translation().Distance(p2d).value();
    if (d < best_dist) {
      best_dist = d;
      closest_idx = i;
    }
  }

  if (prev_closest_idx_.has_value() &&
      closest_idx < static_cast<int>(*prev_closest_idx_)) {
    reset();
    return {.vx = 0.0, .vy = 0.0, .done = false};
  }
  if (best_dist > errorThreshold_) {
    resetPath();
    return {.vx = last_vx_, .vy = last_vy_, .done = false};
  }

  prev_closest_idx_ = closest_idx;

  if (closest_idx >= static_cast<int>(spline_->params.size())) {
    closest_idx = static_cast<int>(spline_->params.size()) - 1;
  }

  // skip index 0 since its speed is 0
  int profileidx =
      std::min(std::max(closest_idx, 1), static_cast<int>(profile.size()) - 1);
  double cx = points[closest_idx].X().value();
  double cy = points[closest_idx].Y().value();
  double vx = profile[profileidx].first + kp_ * (cx - current_pose.X().value());
  double vy =
      profile[profileidx].second + kp_ * (cy - current_pose.Y().value());
  last_vx_ = vx;
  last_vy_ = vy;

  return {.vx = vx, .vy = vy, .done = false};
}

}  // namespace pathing
