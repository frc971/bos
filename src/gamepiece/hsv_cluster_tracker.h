#pragma once

#include "src/camera/camera_constants.h"
#include "src/utils/disjoint_set_union.h"

#include <frc/geometry/Translation2d.h>

namespace gamepiece {

using kmeans_cluster_t = struct KMeansCluster {
  cv::Point2f centroid;
  cv::Mat covar;
  std::vector<cv::Point2f> img_points;
  std::optional<frc::Translation2d> camera_relative_translation = std::nullopt;
};

class HSVClusterTracker {
 public:
  explicit HSVClusterTracker(const camera::camera_constant_t& camera);
  void ProcessFrame(const cv::Mat& frame);
  [[nodiscard]] auto GetClusters() const
      -> const std::vector<kmeans_cluster_t>*;
  void HSVThreshold(const cv::Mat& img);

 private:
  [[nodiscard]] auto KMeans(
      const std::vector<cv::Point2f>& data_points, int k,
      const std::vector<kmeans_cluster_t>& initial_clusters) const
      -> std::vector<kmeans_cluster_t>;
  [[nodiscard]] auto ClusterDistance(const kmeans_cluster_t& cluster) const
      -> frc::Translation2d;
  // must be passed in undistorted convention (normalized and centered)
  [[nodiscard]] auto UndistortedPointOffset(const cv::Point2f& point,
                                            float world_relative_vertical) const
      -> std::optional<frc::Translation2d>;
  [[nodiscard]] auto ClustersOverlap(const kmeans_cluster_t& first,
                                     const kmeans_cluster_t& second) const
      -> bool;
  auto MergeOverlappingClusters(
      const std::vector<kmeans_cluster_t>& unfiltered_clusters)
      -> std::vector<kmeans_cluster_t>;

  int active_cluster_count_ = 20;
  std::vector<cv::Point2f> thresholded_points_;
  std::vector<kmeans_cluster_t> clusters_;
  utils::DisjointSetUnion cluster_dsu_;
  const camera::camera_constant_t camera_constant_;
  cv::Mat hsv_image_;
  cv::Mat hsv_masked_;
  cv::Mat camera_intrinsics_;
  cv::Mat distortion_coeffs_;
  cv::Mat camera_extrinsics_wpi_;
  cv::Mat camera_extrinsics_cv_;
  static constexpr std::pair<int, int> hsv_color_range{18, 30};
  static constexpr int minimum_saturation{180};
  static constexpr float max_merge_distance_m{0.5f};
  const size_t min_pixels_per_cluster_;
  static constexpr float min_pixels_per_cluster_image_px_ratio{0.01f};
  static constexpr float horizon_distance_tolerance{0.01f};
  cv::Mat camera_origin_;
};

}  // namespace gamepiece
