#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

namespace gamepiece {

// rotation is counter-clockwise in radians. semi_axes contains the two radii,
// not the full width and height.
struct Ellipse {
  cv::Point2d center;
  cv::Size2d semi_axes;
  double rotation = 0.0;
};

auto ellipse_intersections(const Ellipse& first, const Ellipse& second)
    -> std::vector<cv::Point2d>;

auto ellipse_overlap_area(const Ellipse& first, const Ellipse& second)
    -> double;

}  // namespace gamepiece
