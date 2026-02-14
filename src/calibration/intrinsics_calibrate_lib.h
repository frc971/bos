#pragma once

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include "src/utils/pch.h"

namespace calibration {

using json = nlohmann::json;
using detection_result_t = struct DetectionResult {
  cv::Mat currentCharucoCorners;
  cv::Mat currentCharucoIds;
  std::vector<cv::Point3f> currentObjectPoints;
  std::vector<cv::Point2f> currentImagePoints;
};

constexpr static int ksquares_x = 12;
constexpr static int ksquares_y = 9;
constexpr static float ksquares_length = 0.025;
constexpr static float kpixel_per_square = 128;
constexpr static float kmarker_length = 0.020;
constexpr static int kmargin_squares = 0;

auto CreateDetector(const cv::aruco::Dictionary& dictionary,
                    int squares_x = ksquares_x, int squares_y = ksquares_y,
                    float squares_length = ksquares_length,
                    float pixel_per_square = kpixel_per_square,
                    float marker_length = kmarker_length)
    -> cv::aruco::CharucoDetector;

auto IntrinsicsToJson(cv::Mat cameraMatrix, cv::Mat distCoeffs) -> json;
auto GenerateBoard(const cv::aruco::CharucoBoard& board,
                   int squares_x = ksquares_x, int squares_y = ksquares_y,
                   float pixel_per_square = kpixel_per_square,
                   int margin_squares = kmargin_squares) -> cv::Mat;

auto DetectCharucoBoard(cv::Mat& frame,
                        const cv::aruco::CharucoDetector& detector)
    -> detection_result_t;

auto DrawDetectionResult(cv::Mat& frame,
                         const detection_result_t& detection_result) -> cv::Mat;

auto CalibrateCamera(const std::vector<detection_result_t>& detection_results,
                     cv::Size image_size, cv::Mat& camera_matrix,
                     cv::Mat& distortion_coefficiants) -> double;

}  // namespace calibration
