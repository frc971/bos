#pragma once

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace calibration {

using json = nlohmann::json;
using detection_result_t = struct DetectionResult {
  cv::Mat currentCharucoCorners;
  cv::Mat currentCharucoIds;
  std::vector<cv::Point3f> currentObjectPoints;
  std::vector<cv::Point2f> currentImagePoints;
};

const static int ksquares_x = 12;
const static int ksquares_y = 9;
const static float ksquares_length = 0.025;
const static float kpixel_per_square = 128;
const static float kmarker_length = 0.020;
const static int kmargin_squares = 0;

auto CreateDetector(const cv::aruco::Dictionary& dictionary,
                    int squares_x = ksquares_x, int squares_y = ksquares_y,
                    float squares_length = ksquares_length,
                    float pixel_per_square = kpixel_per_square,
                    float marker_length = kmarker_length)
    -> cv::aruco::CharucoDetector;

auto intrisincs_to_json(cv::Mat cameraMatrix, cv::Mat distCoeffs) -> json;
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
