#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"
constexpr int kimage_tag_width = 10;
constexpr int kimage_tag_height = 10;
constexpr std::array<int, 2> ktag_ids = {15, 31};
constexpr double kimage_width = 20;
constexpr double kimage_height = 20;
constexpr double krotation = -1.0;
constexpr double ktag_offset = 0;

auto main() -> int {
  // Bottom left, bottom right, top right, top left
  const std::array<cv::Point2f, 4> image_points = {
      cv::Point2f(kimage_width / 2.0 - kimage_tag_width / 2.0 + ktag_offset,
                  kimage_height / 2.0 + kimage_tag_height / 2.0),
      cv::Point2f(
          kimage_width / 2.0 + kimage_tag_width / 2.0 - krotation + ktag_offset,
          kimage_height / 2.0 + kimage_tag_height / 2.0 - krotation),
      cv::Point2f(
          kimage_width / 2.0 + kimage_tag_width / 2.0 - krotation + ktag_offset,
          kimage_height / 2.0 - kimage_tag_height / 2.0),
      cv::Point2f(kimage_width / 2.0 - kimage_tag_width / 2.0 + ktag_offset,
                  kimage_height / 2.0 - kimage_tag_height / 2.0 - krotation)};
  localization::SquareSolver solver(camera::Camera::DUMMY_CAMERA);
  for (auto& point : image_points) {
    std::cout << point << std::endl;
  }

  for (const int id : ktag_ids) {
    const localization::tag_detection_t fake_detection{.tag_id = id,
                                                       .corners = image_points};
    const std::vector<localization::tag_detection_t> fake_detections{
        fake_detection};
    std::cout << id << ":\n"
              << solver.EstimatePosition(fake_detections)[0] << std::endl;
  }

  return 0;
}
