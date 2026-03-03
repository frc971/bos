#include "src/localization/opencv_apriltag_detector.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "src/utils/constants_from_json.h"
#include "src/utils/log.h"

namespace localization {

namespace {

// Build an ArucoDetector configured for the AprilTag 36h11 family with
// parameters tuned for robustness on typical FRC field distances.
auto MakeDetector() -> cv::aruco::ArucoDetector {
  // DICT_APRILTAG_36h11 is the OpenCV dictionary that corresponds to the
  // tag36h11 family used on the FRC field.
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

  cv::aruco::DetectorParameters params;
  // Allow the adaptive threshold window to scale with the image so that tags
  // at varying distances are reliably segmented.
  params.adaptiveThreshWinSizeMin = 3;
  params.adaptiveThreshWinSizeMax = 53;
  params.adaptiveThreshWinSizeStep = 10;
  // Tighten the minimum marker perimeter relative to image size so that tiny,
  // unreliable blobs are rejected early.
  params.minMarkerPerimeterRate = 0.02;
  params.maxMarkerPerimeterRate = 4.0;
  // Use corner refinement for sub-pixel accuracy, which improves downstream
  // PnP accuracy.
  params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params.cornerRefinementWinSize = 5;
  params.cornerRefinementMaxIterations = 30;
  params.cornerRefinementMinAccuracy = 0.1;

  return {dictionary, params};
}

}  // namespace

OpenCVAprilTagDetector::OpenCVAprilTagDetector(const nlohmann::json& intrinsics)
    : camera_matrix_(utils::CameraMatrixFromJson<cv::Mat>(intrinsics)),
      distortion_coefficients_(
          utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics)),
      detector_(MakeDetector()) {}

OpenCVAprilTagDetector::OpenCVAprilTagDetector(int image_width,
                                               int image_height,
                                               const nlohmann::json& intrinsics)
    : OpenCVAprilTagDetector(intrinsics) {}

auto OpenCVAprilTagDetector::GetTagDetections(
    camera::timestamped_frame_t& timestamped_frame)
    -> std::vector<tag_detection_t> {
  // OpenCV's ArUco detector requires an 8-bit greyscale or colour image.
  cv::Mat gray;
  const cv::Mat& src = timestamped_frame.frame;
  if (src.channels() == 1) {
    gray = src;
  } else if (src.channels() == 3) {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  } else {
    LOG(ERROR) << "OpenCVAprilTagDetector: unsupported number of channels: "
               << src.channels();
    return {};
  }

  // Detect markers.  corners[i] holds the 4 image-plane corners of marker i in
  // clockwise order starting from the top-left corner, matching the convention
  // used by the rest of the codebase.
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> rejected;  // unused but required by API
  detector_.detectMarkers(gray, corners, ids, rejected);

  if (ids.empty()) {
    return {};
  }

  std::vector<tag_detection_t> detections;
  detections.reserve(ids.size());

  for (size_t i = 0; i < ids.size(); ++i) {
    std::array<cv::Point2d, 4> corners_array;
    for (int j = 0; j < 4; ++j) {
      corners_array[j] = cv::Point2d(corners[i][j].x, corners[i][j].y);
    }
    std::array<cv::Point2d, 4> undistorted_corners;
    cv::undistortImagePoints(corners_array, undistorted_corners, camera_matrix_,
                             distortion_coefficients_);
    std::swap(corners_array[0], corners_array[1]);
    std::swap(corners_array[2], corners_array[3]);

    tag_detection_t detection{
        .tag_id = ids[i],
        .corners = corners_array,
        .timestamp = timestamped_frame.timestamp,
        .confidence = 1.0,
    };
    detections.push_back(detection);
  }

  return detections;
}

}  // namespace localization
