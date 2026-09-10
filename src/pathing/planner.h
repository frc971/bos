#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace pathing {

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct Pose {
  Point position;
  double heading = 0.0;
  bool shooter_finished = false;
};

struct EllipseCluster {
  Point center;
  double radius_x = 0.0;
  double radius_y = 0.0;
  double density = 0.0; // 0.0 to 1.0
};

struct FieldModel {
  std::vector<EllipseCluster> clusters;
};

enum class Action {
  Intake,
  Shoot
};

struct PlanningState {
  Pose robot;
  FieldModel field;
  std::vector<Action> actions;
  std::size_t action_index = 0;
  double elapsed_time = 0.0;
};

struct PlannerConfig {
  double auto_duration = 20.0;
  double time_margin = 0.25;
  double max_transit_velocity = 4.0;
  double shooting_velocity_limit = 1.5;
  double merge_distance = 1.5;
  double minimum_density = 0.05;
  Point shooting_point = {0.0, 0.0};
  double shooting_overhead = 0.5;
  double min_shoot_time = 1.5;
  double min_intake_time = 2.0;
};

struct CollectionCandidate {
  std::vector<std::size_t> clusters;
  std::vector<Point> waypoints;
  double expected_fuel = 0.0;
  double travel_time = 0.0;
  double score = -std::numeric_limits<double>::infinity();
};

struct Plan {
  bool valid = false;
  Action action = Action::Intake;
  CollectionCandidate candidate;
  std::vector<Point> waypoints;
  double estimated_duration = 0.0;
  double target_velocity_limit = 4.0;
};

class OnlinePlanner {
 public:
  explicit OnlinePlanner(PlannerConfig config = {}) : config_(config) {}

  Plan PlanNext(const PlanningState& state) const;
  bool ShouldAdvanceAction(const Pose& robot, const Plan& current_plan) const;

 private:
  PlannerConfig config_;

  std::vector<CollectionCandidate> GenerateCandidates(const PlanningState& state) const;
  CollectionCandidate GenerateSingleCluster(const PlanningState& state, std::size_t idx) const;
  std::vector<std::size_t> BuildSweep(const PlanningState& state, std::size_t start_idx) const;
  CollectionCandidate BuildCandidate(const PlanningState& state, const std::vector<std::size_t>& clusters) const;

  double Distance(const Point& a, const Point& b) const;
  double EstimateTravelTime(const Point& start, const std::vector<Point>& waypoints) const;
  double EstimateClusterValue(const EllipseCluster& c) const;
  double MinimumFutureTime(const PlanningState& state) const;
  double Score(const CollectionCandidate& cand, const PlanningState& state) const;
  bool Feasible(const CollectionCandidate& cand, const PlanningState& state) const;
};

} // namespace pathing
