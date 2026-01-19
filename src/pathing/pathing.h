#pragma once

#include <frc/geometry/Pose2d.h>
#include <opencv2/opencv.hpp>
#include <vector>

struct Node {
  int x;
  int y;
  bool walkable;
  Node* parent;
  bool visited;
};

auto BFS(std::vector<std::vector<Node>>& grid, Node* start, Node* target)
    -> std::vector<Node*>;

auto constuctLinePath(cv::Mat& canvas, std::vector<Node*> path)
    -> std::vector<std::pair<int, int>>;

auto clampedUniformKnotVector(double k, double p) -> std::vector<double>;

auto basisFunction(double i, double p, double t,
                   const std::vector<double>& knots) -> double;

auto getSplinePoint(double t, const std::vector<std::pair<int, int>>& points,
                    const std::vector<double>& knots, int p)
    -> std::pair<double, double>;

auto createSpline(Node* start, Node* target) -> std::vector<frc::Pose2d>;
