#pragma once

#include <frc/geometry/Translation3d.h>
#include <opencv2/core/mat.hpp>
namespace gamepiece {
using kmeans_cluster_t = struct KMeansCluster {
  cv::Point2d centroid;
  cv::Mat covar;
  std::vector<cv::Point2d> img_points;
};

void hsv_threshold(const cv::Mat& img, cv::Mat& out,
                   const std::pair<int, int>& h_range,
                   const std::pair<int, int>& s_range,
                   const cv::Mat& camera_intrinsics,
                   const cv::Mat& distortion_coeffs = cv::Mat());
auto kmeans(const cv::Mat& hsv_img, int k, double x_weight = 1)
    -> std::vector<kmeans_cluster_t>;
// expects the offsets to be in the format output by cluster_distance
auto eliminate_overlapping_clusters(
    const std::vector<kmeans_cluster_t>& unfiltered_clusters)
    -> std::vector<kmeans_cluster_t>;
auto clusters_overlap(const kmeans_cluster_t k1, const kmeans_cluster_t& k2)
    -> bool;
// returns offset in world coordinates WITHOUT CAMERA ROTATION
auto cluster_distance(const std::vector<kmeans_cluster_t>& clusters,
                      const cv::Mat& camera_extrinsics,
                      const cv::Mat& camera_intrinsics,
                      const cv::Mat& distortion_coeffs = cv::Mat())
    -> frc::Translation2d;
}  // namespace gamepiece
