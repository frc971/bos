#include "src/gamepiece/hsv_cluster_tracker.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <numbers>
#include <random>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

auto RandomClusterColor(std::mt19937& random_generator) -> cv::Scalar {
  std::uniform_int_distribution<int> hue_distribution(0, 179);
  const cv::Mat hsv_pixel(
      1, 1, CV_8UC3,
      cv::Scalar{static_cast<double>(hue_distribution(random_generator)), 220.0,
                 255.0});
  cv::Mat bgr_pixel;
  cv::cvtColor(hsv_pixel, bgr_pixel, cv::COLOR_HSV2BGR);
  const cv::Vec3b color = bgr_pixel.at<cv::Vec3b>(0, 0);
  return {static_cast<double>(color[0]), static_cast<double>(color[1]),
          static_cast<double>(color[2])};
}

auto ClusterEllipse(const gamepiece::kmeans_cluster_t& cluster)
    -> cv::RotatedRect {
  cv::Mat covariance;
  cluster.covar.convertTo(covariance, CV_64F);
  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  cv::eigen(covariance, eigenvalues, eigenvectors);

  constexpr double kStandardDeviationScale = 2.0;
  constexpr double kMinimumDiameter = 2.0;
  const auto ellipse_diameter = [kMinimumDiameter, kStandardDeviationScale](
                                    const double eigenvalue) {
    return std::max(
        2.0 * kStandardDeviationScale * std::sqrt(std::max(eigenvalue, 0.0)),
        kMinimumDiameter);
  };
  const double angle =
      std::atan2(eigenvectors.at<double>(0, 1), eigenvectors.at<double>(0, 0)) *
      180.0 / std::numbers::pi;
  return {cluster.centroid,
          {static_cast<float>(ellipse_diameter(eigenvalues.at<double>(0, 0))),
           static_cast<float>(ellipse_diameter(eigenvalues.at<double>(1, 0)))},
          static_cast<float>(angle)};
}

auto DrawClusterIndex(cv::Mat& image, const std::size_t cluster_index,
                      const cv::Point& centroid) -> void {
  const std::string label = std::to_string(cluster_index);
  constexpr double kFontScale = 0.8;
  constexpr int kTextThickness = 2;
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(
      label, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kTextThickness, &baseline);
  const cv::Point text_origin{centroid.x - text_size.width / 2,
                              centroid.y + text_size.height / 2};

  cv::putText(image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, kFontScale,
              cv::Scalar{0, 0, 0}, kTextThickness + 4, cv::LINE_AA);
  cv::putText(image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, kFontScale,
              cv::Scalar{255, 255, 255}, kTextThickness, cv::LINE_AA);
}

auto InputFramePath() -> std::filesystem::path {
  if (const char* configured_path = std::getenv("HSV_KMEANS_INPUT_FRAME");
      configured_path != nullptr && configured_path[0] != '\0') {
    return configured_path;
  }
  return std::filesystem::path(BOS_SOURCE_DIR) / "frames" / "frame_007888.jpg";
}

auto OutputPath() -> std::filesystem::path {
  if (const char* configured_path = std::getenv("HSV_KMEANS_VISUAL_OUTPUT");
      configured_path != nullptr && configured_path[0] != '\0') {
    return configured_path;
  }
  return std::filesystem::path(BOS_SOURCE_DIR) / "visualizations" /
         "hsv_cluster_tracker_post_merge_test.jpg";
}

TEST(HSVClusterVisualizationTest, DrawsClustersFromRealCameraFrame) {
  const std::filesystem::path input_path = InputFramePath();
  ASSERT_TRUE(std::filesystem::is_regular_file(input_path))
      << "Real HSV test frame does not exist: " << input_path;

  const cv::Mat frame = cv::imread(input_path.string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(frame.empty()) << "Could not decode test frame: " << input_path;

  camera::camera_constant_t camera{
      .name = "real_hsv_visualization",
      .frame_width = static_cast<uint>(frame.cols),
      .frame_height = static_cast<uint>(frame.rows)};
  gamepiece::HSVClusterTracker tracker(camera);
  // Prime the tracker with the same captured frame so the visualization shows
  // the post-merge state after its temporal cluster count has grown.
  tracker.ProcessFrame(frame);
  tracker.ProcessFrame(frame);

  const auto* clusters = tracker.GetClusters();
  ASSERT_FALSE(clusters->empty())
      << "The real frame produced no HSV clusters: " << input_path;

  cv::Mat visualization = frame.clone();
  constexpr std::mt19937::result_type kClusterColorSeed = 0x4B4D4541;
  std::mt19937 random_generator(kClusterColorSeed);
  for (std::size_t cluster_index = 0; cluster_index < clusters->size();
       ++cluster_index) {
    const gamepiece::kmeans_cluster_t& cluster = clusters->at(cluster_index);
    const cv::Scalar color = RandomClusterColor(random_generator);
    const cv::Vec3b pixel_color{static_cast<uchar>(color[0]),
                                static_cast<uchar>(color[1]),
                                static_cast<uchar>(color[2])};

    for (const cv::Point2d& point : cluster.img_points) {
      const cv::Point pixel{static_cast<int>(std::lround(point.x)),
                            static_cast<int>(std::lround(point.y))};
      if (pixel.x >= 0 && pixel.x < visualization.cols && pixel.y >= 0 &&
          pixel.y < visualization.rows) {
        visualization.at<cv::Vec3b>(pixel) = pixel_color;
      }
    }

    const cv::RotatedRect ellipse = ClusterEllipse(cluster);
    cv::ellipse(visualization, ellipse, cv::Scalar{0, 0, 0}, 6, cv::LINE_AA);
    cv::ellipse(visualization, ellipse, color, 2, cv::LINE_AA);

    const cv::Point centroid{static_cast<int>(std::lround(cluster.centroid.x)),
                             static_cast<int>(std::lround(cluster.centroid.y))};
    DrawClusterIndex(visualization, cluster_index, centroid);
  }

  const std::filesystem::path output_path = OutputPath();
  ASSERT_TRUE(cv::imwrite(output_path.string(), visualization))
      << "Could not write HSV cluster visualization to " << output_path;
}

}  // namespace
