#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat initializeGrid(const std::vector<std::vector<bool>>& gridData);

auto BFS(const cv::Mat& grid, std::pair<int, int> start,
         std::pair<int, int> target) -> std::vector<std::pair<int, int>>;

auto createSpline(cv::Mat& grid, int sx, int sy, int tx, int ty,
                  double nodeSize) -> std::vector<frc::Pose2d>;