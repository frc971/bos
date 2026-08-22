#pragma once

#include "src/camera/camera_constants.h"
#include "src/utils/disjoint_set_union.h"

#include <frc/geometry/Translation2d.h>

namespace gamepiece {

struct KMeansCluster {
  cv::Point2d centroid;
  cv::Mat covar;
  std::vector<cv::Point2d> img_points;
};

using kmeans_cluster_t = KMeansCluster;

class HSVClusterTracker {
 public:
  explicit HSVClusterTracker(const camera::camera_constant_t& camera);
  void ProcessFrame(const cv::Mat& frame);
  [[nodiscard]] auto GetClusters() const
      -> const std::vector<kmeans_cluster_t>*;
  void HSVThreshold(const cv::Mat& img);

 private:
  auto KMeans(const std::vector<cv::Point2d>& data_points, int k,
              double x_weight,
              const std::vector<kmeans_cluster_t>& initial_clusters) const
      -> std::vector<kmeans_cluster_t>;
  auto ClusterDistance(const kmeans_cluster_t& cluster) const
      -> frc::Translation2d;
  auto ClustersOverlap(const kmeans_cluster_t& first,
                       const kmeans_cluster_t& second) const -> bool;
  auto MergeOverlappingClusters(
      const std::vector<kmeans_cluster_t>& unfiltered_clusters)
      -> std::vector<kmeans_cluster_t>;

  int active_cluster_count_ = 20;
  std::vector<cv::Point2d> thresholded_points_;
  std::vector<kmeans_cluster_t> clusters_;
  utils::DisjointSetUnion cluster_dsu_;
  const camera::camera_constant_t camera_constant_;
  cv::Mat hsv_image_;
  cv::Mat hsv_masked_;
  cv::Mat camera_intrinsics_;
  cv::Mat distortion_coeffs_;
  cv::Mat camera_extrinsics_;
  static constexpr std::pair<int, int> hsv_color_range{18, 30};
  static constexpr int minimum_saturation{180};
  static constexpr double max_merge_distance_m{0.5};
};

}  // namespace gamepiece
