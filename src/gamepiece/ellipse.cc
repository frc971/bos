#include "src/gamepiece/ellipse.h"

#include <absl/log/check.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <unsupported/Eigen/Polynomials>

namespace gamepiece {
namespace {

constexpr double kGeometryTolerance = 1e-8;

struct EllipseData {
  Eigen::Vector2d center;
  Eigen::Vector2d semi_major_axis_vector;
  Eigen::Vector2d semi_minor_axis_vector;
  Eigen::Matrix2d test_on_ellipse;
  Eigen::Matrix2d rotation;
  double area, a, b;
};

auto GetEllipseData(const Ellipse& ellipse) -> EllipseData {
  const double a = ellipse.semi_axes.width;
  const double b = ellipse.semi_axes.height;
  if (!(a > 0.0) || !(b > 0.0) || !std::isfinite(a) || !std::isfinite(b) ||
      !std::isfinite(ellipse.center.x) || !std::isfinite(ellipse.center.y) ||
      !std::isfinite(ellipse.rotation)) {
    throw std::invalid_argument(
        "Ellipse values must be finite and radii positive");
  }

  const double cosine = std::cos(ellipse.rotation);
  const double sine = std::sin(ellipse.rotation);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;

  EllipseData result;
  result.center = {ellipse.center.x, ellipse.center.y};
  result.semi_major_axis_vector = rotation.col(0) * a;
  result.semi_minor_axis_vector = rotation.col(1) * b;
  result.test_on_ellipse =
      rotation * Eigen::Vector2d(1.0 / (a * a), 1.0 / (b * b)).asDiagonal() *
      rotation.transpose();
  result.rotation = rotation;
  result.area = std::numbers::pi * a * b;
  result.a = a;
  result.b = b;
  return result;
}

auto NormalizedDistanceSquared(const Eigen::Vector2d& point,
                               const EllipseData& ellipse) -> double {
  const Eigen::Vector2d offset = point - ellipse.center;
  return offset.dot(ellipse.test_on_ellipse * offset);
}

auto SameEllipse(const EllipseData& first, const EllipseData& second) -> bool {
  const double center_scale = 1.0 + first.center.norm() + second.center.norm();
  const double matrix_scale =
      1.0 + first.test_on_ellipse.norm() + second.test_on_ellipse.norm();
  return (first.center - second.center).norm() <=
             kGeometryTolerance * center_scale &&
         (first.test_on_ellipse - second.test_on_ellipse).norm() <=
             kGeometryTolerance * matrix_scale;
}

auto QuarticCoefficients(const EllipseData& source_ellipse,
                         const EllipseData& constraint_ellipse,
                         double source_rotation)
    -> Eigen::Matrix<double, 5, 1> {
  const Eigen::Vector2d phase_radial =
      source_ellipse.semi_major_axis_vector * std::cos(source_rotation) +
      source_ellipse.semi_minor_axis_vector * std::sin(source_rotation);

  const Eigen::Vector2d phase_tangent =
      -source_ellipse.semi_major_axis_vector * std::sin(source_rotation) +
      source_ellipse.semi_minor_axis_vector * std::cos(source_rotation);

  const Eigen::Vector2d center_offset =
      source_ellipse.center - constraint_ellipse.center;

  // polynomial a bu cu^2 order
  const std::array<Eigen::Vector2d, 3> numerator_coefficients{
      center_offset + phase_radial,
      2.0 * phase_tangent,
      center_offset - phase_radial,
  };

  Eigen::Matrix<double, 5, 1> quartic = Eigen::Matrix<double, 5, 1>::Zero();

  for (int i = 0; i <= 2; ++i) {
    for (int j = 0; j <= 2; ++j) {
      quartic[i + j] += numerator_coefficients[i].dot(
          constraint_ellipse.test_on_ellipse * numerator_coefficients[j]);
    }
  }

  // from denominator (1 + u^2)^2
  quartic[0] -= 1.0;
  quartic[2] -= 2.0;
  quartic[4] -= 1.0;

  return quartic;
}

auto PointAt(const EllipseData& ellipse, double theta) -> Eigen::Vector2d {
  return ellipse.center + ellipse.semi_major_axis_vector * std::cos(theta) +
         ellipse.semi_minor_axis_vector * std::sin(theta);
}

auto ThetaFromPoint(const EllipseData& ellipse, const Eigen::Vector2d& point)
    -> double {
  const Eigen::Vector2d centered =
      ellipse.rotation.transpose() * (point - ellipse.center);

  return std::atan2(centered.y() / ellipse.b, centered.x() / ellipse.a);
}

auto SectorArea(const EllipseData& ellipse, double angle) -> double {
  return ellipse.a * ellipse.b * 0.5 * std::abs(angle);
}

auto OverlapArea(const EllipseData& ellipse_1, const EllipseData& ellipse_2,
                 const std::vector<cv::Point2d>& intersections) -> double {
  if (intersections.empty() && intersections.size() == 1) {
    return 0;
  } else if (intersections.size() > 2) {
    // not meant to be accurate, if there are 2+ intersection points the clusters
    // should be merged anyway
    return ellipse_1.area + ellipse_2.area;
  }
  double ellipse_1_lower_bound = ThetaFromPoint(ellipse_1, intersections[0]);
  double ellipse_1_upper_bound = ThetaFromPoint(ellipse_1, intersections[1]);
  double angle_1 = ellipse_1_upper_bound - ellipse_1_lower_bound;
  double ellipse_1_curved_area =
      SectorArea(ellipse_1, angle_1) -
      0.5 * ellipse_1.a * ellipse_1.b * std::sin(angle_1);

  double ellipse_2_lower_bound = ThetaFromPoint(ellipse_2, intersections[0]);
  double ellipse_2_upper_bound = ThetaFromPoint(ellipse_2, intersections[1]);
  double angle_2 = ellipse_2_upper_bound - ellipse_2_lower_bound;
  double ellipse_2_curved_area =
      SectorArea(ellipse_2, angle_2) -
      0.5 * ellipse_2.a * ellipse_2.b * std::sin(angle_2);
  return ellipse_1_curved_area + ellipse_2_curved_area;
}

}  // namespace

auto ellipse_intersections(const Ellipse& first, const Ellipse& second)
    -> std::vector<cv::Point2d> {
  const EllipseData first_data = GetEllipseData(first);
  const EllipseData second_data = GetEllipseData(second);
  if (SameEllipse(first_data, second_data)) {
    return {};
  }

  // Choosing a phase whose opposite point is not an intersection keeps the
  // tan-half-angle polynomial genuinely quartic, as required by the fixed-size
  // Eigen solver. Maximizing the leading term also improves conditioning.
  constexpr std::array<double, 8> phases{0.0,  0.37, 0.79, 1.21,
                                         1.63, 2.05, 2.47, 2.89};
  Eigen::Matrix<double, 5, 1> coefficients;
  double phase = 0.0;
  double best_leading_ratio = -1.0;
  for (const double candidate_phase : phases) {
    const auto candidate =
        QuarticCoefficients(first_data, second_data, candidate_phase);
    const double ratio = std::abs(candidate[4]) / (candidate.norm() + 1e-300);
    if (ratio > best_leading_ratio) {
      best_leading_ratio = ratio;
      coefficients = candidate;
      phase = candidate_phase;
    }
  }

  if (best_leading_ratio <= 1e-12) {
    return {};
  }
  coefficients /= coefficients.cwiseAbs().maxCoeff();
  Eigen::PolynomialSolver<double, 4> solver(coefficients);

  std::vector<cv::Point2d> intersections;
  for (const std::complex<double>& root : solver.roots()) {
    if (std::abs(root.imag()) > 1e-6 * (1.0 + std::abs(root.real()))) {
      continue;
    }
    const double theta = phase + 2.0 * std::atan(root.real());
    const Eigen::Vector2d point = PointAt(first_data, theta);
    const double residual =
        std::abs(NormalizedDistanceSquared(point, second_data) - 1.0);
    if (residual > 1e-6) {
      continue;
    }

    const cv::Point2d result(point.x(), point.y());
    const double scale = 1.0 + point.norm();
    const bool duplicate =
        std::any_of(intersections.begin(), intersections.end(),
                    [&](const cv::Point2d& old) {
                      return cv::norm(result - old) <= 1e-6 * scale;
                    });
    if (!duplicate) {
      intersections.push_back(result);
    }
  }
  return intersections;
}

auto ellipse_overlap_area(const Ellipse& first, const Ellipse& second)
    -> double {
  const EllipseData first_data = GetEllipseData(first);
  const EllipseData second_data = GetEllipseData(second);
  if (SameEllipse(first_data, second_data)) {
    return first_data.area;
  }

  const std::vector<cv::Point2d> intersections =
      ellipse_intersections(first, second);
  // With zero or one distinct boundary intersection the ellipses are disjoint,
  // tangent, or one contains the other.
  if (intersections.size() <= 1) {
    const bool first_center_inside =
        NormalizedDistanceSquared(first_data.center, second_data) <= 1.0;
    const bool second_center_inside =
        NormalizedDistanceSquared(second_data.center, first_data) <= 1.0;
    if (first_center_inside && second_center_inside) {
      return std::min(first_data.area, second_data.area);
    }
    if (first_center_inside) {
      return first_data.area;
    }
    if (second_center_inside) {
      return second_data.area;
    }
    return 0.0;
  }

  // Green's theorem integrates the inside arcs of both boundaries. For two
  // intersections this is exactly the two ellipse-sector integrals minus the
  // two center-to-chord triangles; it also handles four intersections.
  double area = OverlapArea(first_data, second_data, intersections) +
                OverlapArea(second_data, first_data, intersections);
  area = std::abs(area);
  return std::clamp(area, 0.0, std::min(first_data.area, second_data.area));
}

}  // namespace gamepiece
