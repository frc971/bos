#pragma once

#include <frc/geometry/Pose2d.h>
#include <opencv2/opencv.hpp>
#include <vector>

auto initializeGrid(const std::vector<std::vector<bool>>& gridData) -> cv::Mat;

auto BFS(const cv::Mat& grid, std::pair<int, int> start,
         std::pair<int, int> target) -> std::vector<std::pair<int, int>>;

auto constructLinePath(cv::Mat& canvas, std::vector<std::pair<int, int>> path)
    -> std::vector<std::pair<int, int>>;

auto clampedUniformKnotVector(double k, double p) -> std::vector<double>;

auto basisFunction(double i, double p, double t,
                   const std::vector<double>& knots) -> double;

auto getSplinePoint(double t, const std::vector<std::pair<int, int>>& points,
                    const std::vector<double>& knots, int p)
    -> std::pair<double, double>;

auto createSpline(cv::Mat& grid, int start_x, int start_y, int target_x,
                  int target_y, double nodeSizeMeters)
    -> std::vector<frc::Pose2d>;
