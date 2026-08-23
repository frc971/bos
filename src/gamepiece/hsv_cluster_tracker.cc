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

namespace {

constexpr int kAdditionalClusters = 3;

auto SquaredDistance(const cv::Point2f& first, const cv::Point2f& second)
    -> float {
  const float x = first.x - second.x;
  const float y = first.y - second.y;
  return x * x + y * y;
}

auto Covariance(const std::vector<cv::Point2f>& points) -> cv::Mat {
  cv::Point2f mean{0.0f, 0.0f};
  for (const cv::Point2f& point : points) {
    mean += point;
  }
  mean /= static_cast<float>(points.size());

  cv::Mat covariance = cv::Mat::zeros(2, 2, CV_32F);
  for (const cv::Point2f& point : points) {
    const cv::Point2f offset = point - mean;
    covariance.at<float>(0, 0) += offset.x * offset.x;
    covariance.at<float>(0, 1) += offset.x * offset.y;
    covariance.at<float>(1, 0) += offset.y * offset.x;
    covariance.at<float>(1, 1) += offset.y * offset.y;
  }
  return covariance / static_cast<float>(points.size());
}

}  // namespace

HSVClusterTracker::HSVClusterTracker(const camera::camera_constant_t& camera)
    : camera_constant_(camera),
      min_pixels_per_cluster_(
          static_cast<int>(camera.frame_height.value_or(800) *
                           camera.frame_width.value_or(1280) *
                           min_pixels_per_cluster_image_px_ratio)) {
  if (!camera_constant_.intrinsics_path.has_value()) {
    LOG(FATAL) << "Cannot run gamepiece without intrinsics";
  }
  if (!camera_constant_.extrinsics_path.has_value()) {
    LOG(FATAL) << "Cannot run gamepiece without extrinsics";
  }
  const nlohmann::json intrinsics =
      utils::ReadIntrinsics(*camera_constant_.intrinsics_path);
  camera_intrinsics_ = utils::CameraMatrixFromJson<cv::Mat>(intrinsics);
  distortion_coeffs_ =
      utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics);

  const nlohmann::json extrinsics =
      utils::ReadExtrinsics(*camera_constant_.extrinsics_path);
  const cv::Mat camera_extrinsics = utils::EigenToCvMat(
      utils::ExtrinsicsJsonToCameraToRobot(extrinsics).ToMatrix());
  camera_extrinsics.convertTo(camera_extrinsics_wpi_, CV_32F);
  camera_extrinsics_cv_ = camera_extrinsics.clone();
  utils::ChangeBasis(camera_extrinsics_cv_, utils::WPI_TO_CV);
  camera_extrinsics_cv_.convertTo(camera_extrinsics_cv_, CV_32F);
  camera_origin_ =
      camera_extrinsics_cv_ * (cv::Mat_<float>(4, 1) << 0.0f, 0.0f, 0.0f, 1.0f);
}

void HSVClusterTracker::ProcessFrame(const cv::Mat& frame) {
  const std::vector<kmeans_cluster_t> previous_clusters = clusters_;
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
      {active_cluster_count_, static_cast<int>(thresholded_points_.size()),
       static_cast<int>(previous_clusters.size() + kAdditionalClusters)});
  const std::vector<kmeans_cluster_t> unmerged_clusters =
      KMeans(thresholded_points_, cluster_count, previous_clusters);
  clusters_ = MergeOverlappingClusters(unmerged_clusters);
  for (auto& cluster : clusters_) {
    cluster.camera_relative_translation.emplace(ClusterDistance(cluster));
  }
}

void HSVClusterTracker::HSVThreshold(const cv::Mat& img) {
  thresholded_points_.clear();

  cv::cvtColor(img, hsv_image_, cv::COLOR_BGR2HSV);

  cv::inRange(hsv_image_,
              cv::Scalar(hsv_color_range.first, minimum_saturation, 0),
              cv::Scalar(hsv_color_range.second, 255, 255), hsv_masked_);
  cv::findNonZero(hsv_masked_, thresholded_points_);

  if (!thresholded_points_.empty()) {
    cv::undistortPoints(thresholded_points_, thresholded_points_,
                        camera_intrinsics_, distortion_coeffs_);
  }
}

auto HSVClusterTracker::UndistortedPointOffset(const cv::Point2f& point,
                                               float world_relative_vertical,
                                               bool verbose) const
    -> std::optional<frc::Translation2d> {
  cv::Mat camera_ray = (cv::Mat_<float>(4, 1) << point.x, point.y, 1.0f, 0.0f);
  camera_ray = camera_extrinsics_cv_ * camera_ray;

  const float ray_y = camera_ray.at<float>(1, 0);
  if (ray_y <= std::numeric_limits<float>::epsilon()) {
    return std::nullopt;
  }
  const float scale =
      (world_relative_vertical - camera_origin_.at<float>(1, 0)) / ray_y;
  const cv::Mat floor_relative_offset = scale * camera_ray;
  if (cv::norm(floor_relative_offset) > 16) {  // TODO get from field constants
    return std::nullopt;
  }
  if (verbose) {
    LOG(INFO) << "Scale: " << scale << " ray_y " << ray_y;
  }
  return std::make_optional<frc::Translation2d>(
      {units::meter_t{floor_relative_offset.at<float>(2, 0)},
       units::meter_t{-floor_relative_offset.at<float>(0, 0)}});
}

auto HSVClusterTracker::KMeans(
    const std::vector<cv::Point2f>& data_points, const int k,
    const std::vector<kmeans_cluster_t>& initial_clusters) const
    -> std::vector<kmeans_cluster_t> {
  if (data_points.empty() || k <= 0 ||
      k > static_cast<int>(data_points.size())) {
    throw std::invalid_argument("Invalid HSV KMeans configuration");
  }

  std::vector<cv::Point2f> scaled_points;
  std::vector<size_t> used_point_indices;
  scaled_points.reserve(data_points.size());
  float average_point_depth = 0;
  for (size_t i = 0; i < data_points.size(); i++) {
    // inaccurate for most points because this assumes they're on the floor, may change later
    const std::optional<frc::Translation2d> point_offset =
        UndistortedPointOffset(data_points[i], 0);
    if (!point_offset.has_value()) {
      continue;  // to avoid yellow in the stands, which is above the field hoizon line
    }
    const float depth = point_offset.value().Norm().value();
    average_point_depth += depth;
    scaled_points.emplace_back(data_points[i].x, data_points[i].y * depth);
    used_point_indices.push_back(i);
  }
  if (scaled_points.empty()) {
    return {};
  }
  average_point_depth /= used_point_indices.size();

  cv::Mat labels;
  cv::Mat centers;
  const cv::TermCriteria criteria(
      cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-4);

  std::vector<cv::Point2f> initial_centers;
  initial_centers.reserve(k);
  for (const kmeans_cluster_t& cluster : initial_clusters) {
    if (static_cast<int>(initial_centers.size()) == k) {
      break;
    }
    const std::optional<frc::Translation2d> point_offset =
        UndistortedPointOffset(cluster.centroid, 0);
    if (point_offset.has_value()) {
      initial_centers.emplace_back(
          cluster.centroid.x,
          cluster.centroid.y * point_offset.value().Norm().value());
    }
  }

  std::vector<bool> new_centroids(scaled_points.size(), false);
  while (static_cast<int>(initial_centers.size()) < k) {
    std::size_t farthest_point = scaled_points.size();
    float farthest_distance = -1.0f;
    for (std::size_t point_index = 0; point_index < scaled_points.size();
         ++point_index) {
      if (new_centroids[point_index]) {
        continue;
      }

      float distance_to_nearest_center = std::numeric_limits<float>::max();
      for (const cv::Point2f& center : initial_centers) {
        distance_to_nearest_center =
            std::min(distance_to_nearest_center,
                     SquaredDistance(scaled_points[point_index], center));
      }
      if (initial_centers.empty() ||
          distance_to_nearest_center > farthest_distance) {
        farthest_point = point_index;
        farthest_distance = distance_to_nearest_center;
      }
    }

    if (farthest_point == scaled_points.size()) {
      break;
    }
    new_centroids[farthest_point] = true;
    initial_centers.push_back(scaled_points[farthest_point]);
  }

  labels = cv::Mat(static_cast<int>(scaled_points.size()), 1, CV_32S);
  std::vector<int> label_counts(k, 0);
  for (int point_index = 0; point_index < labels.rows; ++point_index) {
    int nearest_center = 0;
    float nearest_distance =
        SquaredDistance(scaled_points[point_index], initial_centers.front());
    for (int center_index = 1; center_index < k; ++center_index) {
      const float distance = SquaredDistance(scaled_points[point_index],
                                             initial_centers[center_index]);
      if (distance < nearest_distance) {
        nearest_center = center_index;
        nearest_distance = distance;
      }
    }
    labels.at<int>(point_index, 0) = nearest_center;
    ++label_counts[nearest_center];
  }

  for (int empty_cluster = 0; empty_cluster < k; ++empty_cluster) {
    if (label_counts[empty_cluster] != 0) {
      continue;
    }

    int point_to_reassign = -1;
    float closest_distance = std::numeric_limits<float>::max();
    for (int point_index = 0; point_index < labels.rows; ++point_index) {
      const int current_cluster = labels.at<int>(point_index, 0);
      if (label_counts[current_cluster] <= 1) {
        continue;
      }
      const float distance = SquaredDistance(scaled_points[point_index],
                                             initial_centers[empty_cluster]);
      if (distance < closest_distance) {
        point_to_reassign = point_index;
        closest_distance = distance;
      }
    }

    if (point_to_reassign >= 0) {
      const int old_cluster = labels.at<int>(point_to_reassign, 0);
      labels.at<int>(point_to_reassign, 0) = empty_cluster;
      --label_counts[old_cluster];
      ++label_counts[empty_cluster];
    }
  }

  cv::kmeans(scaled_points, k, labels, criteria, 1,
             cv::KMEANS_USE_INITIAL_LABELS, centers);

  std::vector<cv::Point2f> centroid_sums(k, {0.0f, 0.0f});
  std::vector<int> centroid_counts(k, 0);

  for (int i = 0; i < labels.rows; ++i) {
    const int cluster = labels.at<int>(i, 0);
    centroid_sums[cluster] += data_points[used_point_indices[i]];
    ++centroid_counts[cluster];
  }

  std::vector<cv::Point2f> centroids(k);
  for (int i = 0; i < k; ++i) {
    centroids[i] = centroid_sums[i] / static_cast<float>(centroid_counts[i]);
  }

  std::vector<std::vector<cv::Point2f>> cluster_points(k);
  for (int i = 0; i < labels.rows; ++i) {
    const int cluster = labels.at<int>(i, 0);
    cluster_points.at(cluster).push_back(data_points.at(used_point_indices[i]));
  }

  std::vector<kmeans_cluster_t> clusters;
  clusters.reserve(k);
  for (int i = 0; i < k; ++i) {
    kmeans_cluster_t cluster;
    cluster.centroid = centroids[i];
    cluster.img_points = std::move(cluster_points.at(i));

    if (cluster.img_points.size() > 1) {
      cluster.covar = Covariance(cluster.img_points);
    } else {
      cluster.covar = cv::Mat::eye(2, 2, CV_32F);
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

auto HSVClusterTracker::ClusterDistance(const kmeans_cluster_t& cluster) const
    -> frc::Translation2d {
  if (cluster.img_points.empty()) {
    return {};
  }

  const auto lowest_point =
      std::max_element(cluster.img_points.begin(), cluster.img_points.end(),
                       [](const cv::Point2f& first, const cv::Point2f& second) {
                         return first.y < second.y;
                       });
  const auto offset = UndistortedPointOffset(*lowest_point, 0).value();
  return offset;
}

auto HSVClusterTracker::ClustersOverlap(const kmeans_cluster_t& first,
                                        const kmeans_cluster_t& second) const
    -> bool {
  const auto make_ellipse = [](const kmeans_cluster_t& cluster) -> Ellipse {
    const cv::Mat& covariance = cluster.covar;
    if (covariance.rows != 2 || covariance.cols != 2) {
      throw std::invalid_argument("KMeans covariance must be 2 by 2");
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvectors;
    cv::eigen(covariance, eigenvalues, eigenvectors);
    constexpr float kMinimumRadius = 1e-6f;
    const float major =
        std::sqrt(std::max(eigenvalues.at<float>(0, 0), 0.0f)) + kMinimumRadius;
    const float minor =
        std::sqrt(std::max(eigenvalues.at<float>(1, 0), 0.0f)) + kMinimumRadius;
    const cv::Vec2f major_axis(eigenvectors.at<float>(0, 0),
                               eigenvectors.at<float>(0, 1));
    return {.center = {cluster.centroid.x, cluster.centroid.y},
            .semi_axes = {major, minor},
            .rotation = std::atan2(major_axis[1], major_axis[0])};
  };

  return ellipse_overlap_area(make_ellipse(first), make_ellipse(second)) > 0.0f;
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

    cv::Point2f point_sum{0.0f, 0.0f};
    for (const cv::Point2f& point : merged.img_points) {
      point_sum += point;
    }
    merged.centroid = point_sum / static_cast<float>(merged.img_points.size());

    if (merged.img_points.size() > 1) {
      merged.covar = Covariance(merged.img_points);
    } else {
      merged.covar = cv::Mat::eye(2, 2, CV_32F);
    }
  }

  return merged_clusters;
}

auto HSVClusterTracker::GetClusters() const
    -> const std::vector<kmeans_cluster_t>* {
  return &clusters_;
}

}  // namespace gamepiece
