#include "src/gamepiece/hsv_kmeans.h"
#include <opencv2/imgproc.hpp>

namespace gamepiece {
void hsv_threshold(const cv::Mat& img, std::vector<cv::Point2d>& out,
                   const std::pair<int, int>& h_range,
                   const std::pair<int, int>& s_range) {
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  cv::Mat hsv_masked;
  cv::inRange(hsv, cv::Scalar(h_range.first, s_range.first, 0),
              cv::Scalar(h_range.second, s_range.second, 255), hsv_masked);
  cv::findNonZero(hsv_masked, out);
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
                      const cv::Mat& distortion_coeffs) -> double {}

auto eliminate_overlapping_clusters(
    const std::vector<kmeans_cluster_t>& unfiltered_clusters)
    -> std::vector<kmeans_cluster_t> {}
}  // namespace gamepiece
