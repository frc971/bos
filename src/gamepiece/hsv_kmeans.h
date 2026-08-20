#pragma once

#include <opencv2/core/mat.hpp>
namespace gamepiece {
using kmeans_cluster_t = struct KMeansCluster {
  cv::Point2d centroid;
  cv::Mat covar;
  std::vector<cv::Point2d> img_points;
};

void hsv_threshold(const cv::Mat& img, cv::Mat& out,
                   const std::pair<int, int>& h_range,
                   const std::pair<int, int>& s_range);
auto kmeans(const cv::Mat& hsv_img, int k, double x_weight = 1)
    -> std::vector<kmeans_cluster_t>;
auto eliminate_overlapping_clusters(
    const std::vector<kmeans_cluster_t>& unfiltered_clusters)
    -> std::vector<kmeans_cluster_t>;
auto cluster_distance(const std::vector<kmeans_cluster_t>& clusters,
                      const cv::Mat& camera_extrinsics,
                      const cv::Mat& camera_intrinsics) -> double;
}  // namespace gamepiece
