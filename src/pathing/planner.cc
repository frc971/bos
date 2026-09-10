#include "pathfinding.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace pathing {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Plan OnlinePlanner::PlanNext(const PlanningState& state) const {
  if (state.action_index >= state.actions.size()) {
    return {.valid = false};
  }

  Action current_action = state.actions[state.action_index];
  Plan plan;
  plan.action = current_action;

  // Handle SHOOT Action: Must return to alliance zone with limited velocity
  if (current_action == Action::Shoot) {
    plan.target_velocity_limit = config_.shooting_velocity_limit;
    plan.waypoints.push_back(config_.shooting_point);
    
    double travel_time = EstimateTravelTime(state.robot.position, plan.waypoints);
    plan.estimated_duration = travel_time + config_.shooting_overhead;

    double remaining_time = config_.auto_duration - state.elapsed_time;
    if (plan.estimated_duration <= remaining_time - config_.time_margin) {
      plan.valid = true;
      return plan;
    }
    return {.valid = false}; // Out of time for shooting
  }

  // Handle INTAKE Action: Full transit speed, evaluate dynamic cluster array
  plan.target_velocity_limit = config_.max_transit_velocity;
  auto candidates = GenerateCandidates(state);
  const CollectionCandidate* best = nullptr;

  for (const auto& cand : candidates) {
    if (!Feasible(cand, state)) continue;
    if (best == nullptr || cand.score > best->score) {
      best = &cand;
    }
  }

  if (!best) {
    return {.valid = false}; // No feasible intake targets found within time budget
  }

  plan.valid = true;
  plan.candidate = *best;
  plan.waypoints = best->waypoints;
  plan.estimated_duration = best->travel_time;
  return plan;
}

std::vector<CollectionCandidate> OnlinePlanner::GenerateCandidates(const PlanningState& state) const {
  std::vector<CollectionCandidate> result;
  const auto& clusters = state.field.clusters;

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].density < config_.minimum_density) continue;
    
    // Single cluster option
    result.push_back(GenerateSingleCluster(state, i));
    
    // Multi-cluster greedy sweep option
    auto sweep = BuildSweep(state, i);
    if (sweep.size() > 1) {
      result.push_back(BuildCandidate(state, sweep));
    }
  }
  return result;
}

CollectionCandidate OnlinePlanner::GenerateSingleCluster(const PlanningState& state, std::size_t idx) const {
  return BuildCandidate(state, {idx});
}

std::vector<std::size_t> OnlinePlanner::BuildSweep(const PlanningState& state, std::size_t start_idx) const {
  const auto& clusters = state.field.clusters;
  std::vector<std::size_t> sweep = {start_idx};
  std::vector<bool> used(clusters.size(), false);
  used[start_idx] = true;

  Point current = clusters[start_idx].center;
  while (sweep.size() < 4) { // Cap sweep size to prevent excessive path length
    double best_inc_val = 0.0;
    std::size_t best_j = clusters.size();

    for (std::size_t j = 0; j < clusters.size(); ++j) {
      if (used[j] || clusters[j].density < config_.minimum_density) continue;
      
      double dist = Distance(current, clusters[j].center);
      if (dist > config_.merge_distance) continue;

      double value = EstimateClusterValue(clusters[j]);
      double travel = std::max(dist / config_.max_transit_velocity, 0.01);
      double inc = value / travel;

      if (inc > best_inc_val) {
        best_inc_val = inc;
        best_j = j;
      }
    }

    if (best_j == clusters.size()) break;
    used[best_j] = true;
    sweep.push_back(best_j);
    current = clusters[best_j].center;
  }
  return sweep;
}

CollectionCandidate OnlinePlanner::BuildCandidate(const PlanningState& state, const std::vector<std::size_t>& clusters) const {
  CollectionCandidate cand;
  cand.clusters = clusters;

  for (auto idx : clusters) {
    if (idx < state.field.clusters.size()) {
      cand.waypoints.push_back(state.field.clusters[idx].center);
    }
  }
  // Every intake sweep must terminate by returning to the alliance shooting zone
  cand.waypoints.push_back(config_.shooting_point);

  cand.travel_time = EstimateTravelTime(state.robot.position, cand.waypoints);
  
  double total_fuel = 0.0;
  for (auto idx : clusters) {
    if (idx < state.field.clusters.size()) {
      total_fuel += EstimateClusterValue(state.field.clusters[idx]);
    }
  }
  cand.expected_fuel = total_fuel;
  cand.score = Score(cand, state);
  return cand;
}

double OnlinePlanner::Distance(const Point& a, const Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double OnlinePlanner::EstimateTravelTime(const Point& start, const std::vector<Point>& waypoints) const {
  Point prev = start;
  double time = 0.0;
  for (const auto& pt : waypoints) {
    double d = Distance(prev, pt);
    time += d / std::max(config_.max_transit_velocity, 0.01);
    prev = pt;
  }
  return time;
}

double OnlinePlanner::EstimateClusterValue(const EllipseCluster& c) const {
  double area = kPi * c.radius_x * c.radius_y;
  return area * c.density;
}

double OnlinePlanner::MinimumFutureTime(const PlanningState& state) const {
  double t = 0.0;
  for (std::size_t i = state.action_index + 1; i < state.actions.size(); ++i) {
    if (state.actions[i] == Action::Shoot) {
      t += config_.min_shoot_time;
    } else {
      t += config_.min_intake_time;
    }
  }
  return t;
}

double OnlinePlanner::Score(const CollectionCandidate& cand, const PlanningState& state) const {
  if (cand.travel_time <= 0) return -std::numeric_limits<double>::infinity();
  
  double score = cand.expected_fuel / cand.travel_time;
  if (cand.clusters.size() > 1) score *= 1.10; // Multi-cluster sweep bonus

  // Penalize score if time buffer is getting tight
  double remaining = config_.auto_duration - state.elapsed_time;
  double future = MinimumFutureTime(state);
  double slack = remaining - cand.travel_time - future;
  if (slack < 1.5) {
    score *= 0.60;
  }
  return score;
}

bool OnlinePlanner::Feasible(const CollectionCandidate& cand, const PlanningState& state) const {
  double remaining = config_.auto_duration - state.elapsed_time;
  if (remaining <= 0) return false;

  double future_budget = MinimumFutureTime(state);
  double total_required = cand.travel_time + future_budget + config_.time_margin;

  return total_required <= remaining;
}

bool OnlinePlanner::ShouldAdvanceAction(const Pose& robot, const Plan& current_plan) const {
  if (current_plan.waypoints.empty()) return true;

  if (current_plan.action == Action::Intake) {
    // Advance when robot reaches the final waypoint (back in scoring zone)
    double dist = std::hypot(robot.position.x - current_plan.waypoints.back().x,
                             robot.position.y - current_plan.waypoints.back().y);
    return dist < 0.25;
  } else if (current_plan.action == Action::Shoot) {
    // Advance when shooter mechanism finishes scoring
    return robot.shooter_finished;
  }
  return false;
}

} // namespace pathing
