#pragma once

#include <frc/geometry/Translation2d.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace pathing {

auto InitializeGrid(const std::vector<std::vector<bool>>& gridData) -> cv::Mat;

auto BFS(const cv::Mat& grid, cv::Point2i start, cv::Point2i target,
           bool verbose = false) -> std::vector<cv::Point2i>;

auto ConstructLinePath(cv::Mat& canvas, std::vector<cv::Point2i> path)
    -> std::vector<cv::Point2i>;

auto ClampedUniformKnotVector(size_t num_control_points, uint degree)
    -> std::vector<double>;

auto BasisFunction(size_t index, uint degree, double t,
                   const std::vector<double>& knots) -> double;

auto GetSplinePoint(double t, const std::vector<cv::Point2i>& points,
                    const std::vector<double>& knots, uint degree)
    -> frc::Translation2d;

auto CreateSpline(cv::Mat& grid, int start_x, int start_y, int target_x,
                  int target_y) -> std::vector<frc::Translation2d>;

}  // namespace pathing