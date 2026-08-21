#include "src/gamepiece/hsv_kmeans.h"
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace gamepiece {
void hsv_threshold(const cv::Mat& img, std::vector<cv::Point2d>& out,
                   const std::pair<int, int>& h_range,
                   const std::pair<int, int>& s_range,
                   const cv::Mat& camera_intrinsics,
                   const cv::Mat& distortion_coeffs = cv::Mat()) {
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  cv::Mat hsv_masked;
  cv::inRange(hsv, cv::Scalar(h_range.first, s_range.first, 0),
              cv::Scalar(h_range.second, s_range.second, 255), hsv_masked);
  cv::findNonZero(hsv_masked, out);
  cv::undistortPoints(out, out, camera_intrinsics, distortion_coeffs);
}

auto kmeans(const std::vector<cv::Point2d>& data_points, int k,
            cv::TermCriteria& config, const double x_weight)
    -> std::vector<KMeansCluster> {
  std::vector<cv::Point2d> x_scaled_points = data_points;
  for (auto& img_point : x_scaled_points) {
    img_point.x *= x_weight;
  }
  cv::Mat labels, centers;
  cv::kmeans(data_points, k, labels, config, 10, cv::KMEANS_PP_CENTERS,
             centers);
  for (int i = 0; i < centers.rows; i++) {
    centers.at<float>(i, 0) /= x_weight;
  }
  std::vector<std::vector<cv::Point2d>> cluster_points(k);

  for (int i = 0; i < labels.rows; ++i) {
    const int cluster = labels.at<int>(i);
    cluster_points[cluster].push_back(data_points[i]);
  }

  std::vector<kmeans_cluster_t> clusters;
  clusters.reserve(k);
  for (int i = 0; i < k; i++) {
    kmeans_cluster_t cluster;
    cluster.centroid = centers.at<cv::Point2d>(i);
    cv::calcCovarMatrix(cluster_points[i], cluster.covar, cv::noArray(),
                        cv::COVAR_NORMAL | cv::COVAR_ROWS);
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

auto cluster_distance(const std::vector<kmeans_cluster_t>& clusters,
                      const cv::Mat& camera_extrinsics,
                      const cv::Mat& camera_intrinsics,
                      const cv::Mat& distortion_coeffs) -> frc::Translation2d {
  // estimation of the floor at the lowest point. Needs RIGOROUS testing to ensure that this is an accurate estimation,
  // since we could be seeing balls over the bump and they would be cut off.
  cv::Point2d lowest_point;
  for (const auto& cluster : clusters) {
    for (const auto& point : cluster.img_points) {
      if (point.y < lowest_point.y) {
        lowest_point = point;
      }
    }
  }

  std::vector<cv::Point2d> undistorted_points;
  cv::undistortPoints(std::vector<cv::Point2d>{lowest_point},
                      undistorted_points, camera_intrinsics, distortion_coeffs);
  const cv::Point2d& normalized_point = undistorted_points.front();

  cv::Mat extrinsics;
  camera_extrinsics.convertTo(extrinsics, CV_64F);
  cv::Mat camera_origin = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
  cv::Mat camera_ray = (cv::Mat_<double>(4, 1) << normalized_point.x,
                        normalized_point.y, 1.0, 0.0);
  camera_origin = extrinsics * camera_origin;
  camera_ray = extrinsics * camera_ray;

  const double ray_y = camera_ray.at<double>(1);
  const double scale = -camera_origin.at<double>(1) / ray_y;

  const cv::Mat floor_point = camera_origin + scale * camera_ray;
  const cv::Mat floor_relative_offset = floor_point - camera_origin;
  return frc::Translation2d{
      units::meter_t{floor_relative_offset.at<double>(0)},
      units::meter_t{floor_relative_offset.at<double>(1)}};
}

auto clusters_overlap(const kmeans_cluster_t& k1, const kmeans_cluster_t& k2)
    -> bool {
  const cv::Vec2d offset{k2.centroid.x - k1.centroid.x,
                         k2.centroid.y - k1.centroid.y};

  const double distance = cv::norm(offset);
  if (distance == 0.0) {
    return true;
  }
  const cv::Vec2d u = offset / distance;
  const double k1_variance = u.dot(k1.covar * u);
  const double k2_variance = u.dot(k2.covar * u);

  const double k1_radius = 2.0 * std::sqrt(std::max(0.0, k1_variance));
  const double k2_radius = 2.0 * std::sqrt(std::max(0.0, k2_variance));

  return distance <= k1_radius + k2_radius;
}

auto eliminate_overlapping_clusters(
    const std::vector<kmeans_cluster_t>& unfiltered_clusters,
    const std::vector<frc::Translation2d>& world_relative_cluster_offsets)
    -> std::vector<kmeans_cluster_t> {
  static constexpr double max_cluster_merge_dist_m = 1;
  std::vector<double> distances(unfiltered_clusters.);
}
}  // namespace gamepiece
