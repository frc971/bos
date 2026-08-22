#include "src/gamepiece/hsv_cluster_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "src/gamepiece/ellipse.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace gamepiece {

HSVClusterTracker::HSVClusterTracker(const camera::camera_constant_t& camera)
    : camera_constant_(camera) {
  if (camera_constant_.intrinsics_path.has_value()) {
    const nlohmann::json intrinsics =
        utils::ReadIntrinsics(*camera_constant_.intrinsics_path);
    camera_intrinsics_ = utils::CameraMatrixFromJson<cv::Mat>(intrinsics);
    distortion_coeffs_ =
        utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics);
  }

  if (camera_constant_.extrinsics_path.has_value()) {
    camera_extrinsics_ = utils::EigenToCvMat(
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(*camera_constant_.extrinsics_path))
            .ToMatrix());
  }
}

void HSVClusterTracker::ProcessFrame(const cv::Mat& frame) {
  clusters_.clear();
  thresholded_points_.clear();

  if (frame.empty()) {
    return;
  }

  HSVThreshold(frame);
  if (thresholded_points_.empty()) {
    return;
  }

  const int cluster_count = std::min(
      active_cluster_count_, static_cast<int>(thresholded_points_.size()));
  clusters_ =
      MergeOverlappingClusters(KMeans(thresholded_points_, cluster_count));
}

void HSVClusterTracker::HSVThreshold(const cv::Mat& img) {
  thresholded_points_.clear();

  cv::cvtColor(img, hsv_image_, cv::COLOR_BGR2HSV);

  cv::inRange(hsv_image_,
              cv::Scalar(hsv_color_range.first, minimum_saturation, 0),
              cv::Scalar(hsv_color_range.second, 255, 255), hsv_masked_);
  cv::findNonZero(hsv_masked_, thresholded_points_);

  if (!thresholded_points_.empty() && !camera_intrinsics_.empty()) {
    cv::undistortPoints(thresholded_points_, thresholded_points_,
                        camera_intrinsics_, distortion_coeffs_);
  }
}

auto HSVClusterTracker::KMeans(const std::vector<cv::Point2d>& data_points,
                               const int k, const double x_weight) const
    -> std::vector<kmeans_cluster_t> {
  if (data_points.empty() || k <= 0 ||
      k > static_cast<int>(data_points.size()) || !(x_weight > 0.0) ||
      !std::isfinite(x_weight)) {
    throw std::invalid_argument("Invalid HSV KMeans configuration");
  }

  std::vector<cv::Point2d> scaled_points = data_points;
  for (cv::Point2d& point : scaled_points) {
    point.x *= x_weight;
  }

  cv::Mat labels;
  cv::Mat centers;
  const cv::TermCriteria criteria(
      cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-4);
  cv::kmeans(scaled_points, k, labels, criteria, 10, cv::KMEANS_PP_CENTERS,
             centers);

  std::vector<std::vector<cv::Point2d>> cluster_points(k);
  for (int i = 0; i < labels.rows; ++i) {
    const int cluster = labels.at<int>(i, 0);
    cluster_points.at(cluster).push_back(data_points.at(i));
  }

  std::vector<kmeans_cluster_t> clusters;
  clusters.reserve(k);
  for (int i = 0; i < k; ++i) {
    kmeans_cluster_t cluster;
    cluster.centroid = {centers.at<double>(i, 0) / x_weight,
                        centers.at<double>(i, 1)};
    cluster.img_points = std::move(cluster_points.at(i));

    if (cluster.img_points.size() > 1) {
      cv::calcCovarMatrix(cluster.img_points, cluster.covar, cv::noArray(),
                          cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE);
    } else {
      cluster.covar = cv::Mat::eye(2, 2, CV_64F);
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

auto HSVClusterTracker::ClusterDistance(const kmeans_cluster_t& cluster) const
    -> frc::Translation2d {
  if (cluster.img_points.empty() || camera_intrinsics_.empty() ||
      camera_extrinsics_.empty()) {
    return {};
  }

  const auto lowest_point =
      std::min_element(cluster.img_points.begin(), cluster.img_points.end(),
                       [](const cv::Point2d& first, const cv::Point2d& second) {
                         return first.y < second.y;
                       });
  const cv::Point2d& normalized_point = *lowest_point;

  cv::Mat extrinsics;
  camera_extrinsics_.convertTo(extrinsics, CV_64F);
  cv::Mat camera_origin = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
  cv::Mat camera_ray = (cv::Mat_<double>(4, 1) << normalized_point.x,
                        normalized_point.y, 1.0, 0.0);
  camera_origin = extrinsics * camera_origin;
  camera_ray = extrinsics * camera_ray;

  const double ray_y = camera_ray.at<double>(1, 0);
  if (std::abs(ray_y) <= std::numeric_limits<double>::epsilon()) {
    return {};
  }
  const double scale = -camera_origin.at<double>(1, 0) / ray_y;
  const cv::Mat floor_relative_offset = scale * camera_ray;
  return {units::meter_t{floor_relative_offset.at<double>(0, 0)},
          units::meter_t{floor_relative_offset.at<double>(1, 0)}};
}

auto HSVClusterTracker::ClustersOverlap(const kmeans_cluster_t& first,
                                        const kmeans_cluster_t& second) const
    -> bool {
  const auto make_ellipse = [](const kmeans_cluster_t& cluster) -> Ellipse {
    cv::Mat covariance;
    cluster.covar.convertTo(covariance, CV_64F);
    if (covariance.rows != 2 || covariance.cols != 2) {
      throw std::invalid_argument("KMeans covariance must be 2 by 2");
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvectors;
    cv::eigen(covariance, eigenvalues, eigenvectors);
    constexpr double kMinimumRadius = 1e-6;
    const double major =
        std::sqrt(std::max(eigenvalues.at<double>(0, 0), 0.0)) + kMinimumRadius;
    const double minor =
        std::sqrt(std::max(eigenvalues.at<double>(1, 0), 0.0)) + kMinimumRadius;
    const cv::Vec2d major_axis(eigenvectors.at<double>(0, 0),
                               eigenvectors.at<double>(0, 1));
    return {.center = cluster.centroid,
            .semi_axes = {major, minor},
            .rotation = std::atan2(major_axis[1], major_axis[0])};
  };

  return ellipse_overlap_area(make_ellipse(first), make_ellipse(second)) > 0.0;
}

auto HSVClusterTracker::MergeOverlappingClusters(
    const std::vector<kmeans_cluster_t>& unfiltered_clusters)
    -> std::vector<kmeans_cluster_t> {
  cluster_dsu_.Clear();
  cluster_dsu_.FillSets(unfiltered_clusters.size());
  std::vector<frc::Translation2d> cluster_positions;
  cluster_positions.reserve(unfiltered_clusters.size());
  for (const auto& cluster : unfiltered_clusters) {
    cluster_positions.push_back(ClusterDistance(cluster));
  }

  for (std::size_t first = 0; first < unfiltered_clusters.size(); ++first) {
    for (std::size_t second = first + 1; second < unfiltered_clusters.size();
         ++second) {
      if (cluster_positions[first]
                  .Distance(cluster_positions[second])
                  .value() <= max_merge_distance_m &&
          ClustersOverlap(unfiltered_clusters[first],
                          unfiltered_clusters[second])) {
        cluster_dsu_.Union(first, second);
      }
    }
  }

  std::vector<kmeans_cluster_t> merged_clusters;
  merged_clusters.reserve(cluster_dsu_.ComponentCount());
  std::vector<std::size_t> component_cluster(unfiltered_clusters.size(),
                                             unfiltered_clusters.size());

  for (std::size_t i = 0; i < unfiltered_clusters.size(); ++i) {
    const std::size_t component = cluster_dsu_.Find(i);
    std::size_t& merged_index = component_cluster[component];
    if (merged_index == unfiltered_clusters.size()) {
      merged_index = merged_clusters.size();
      merged_clusters.push_back(unfiltered_clusters[i]);
      merged_clusters.back().img_points.clear();
    }

    auto& merged = merged_clusters[merged_index];
    const auto& points = unfiltered_clusters[i].img_points;
    merged.img_points.insert(merged.img_points.end(), points.begin(),
                             points.end());
  }

  for (kmeans_cluster_t& merged : merged_clusters) {
    if (merged.img_points.empty()) {
      continue;
    }

    cv::Point2d point_sum{0.0, 0.0};
    for (const cv::Point2d& point : merged.img_points) {
      point_sum += point;
    }
    merged.centroid = point_sum / static_cast<double>(merged.img_points.size());

    if (merged.img_points.size() > 1) {
      cv::calcCovarMatrix(merged.img_points, merged.covar, cv::noArray(),
                          cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE);
    } else {
      merged.covar = cv::Mat::eye(2, 2, CV_64F);
    }
  }

  return merged_clusters;
}

auto HSVClusterTracker::GetClusters() const
    -> const std::vector<kmeans_cluster_t>* {
  return &clusters_;
}

}  // namespace gamepiece
