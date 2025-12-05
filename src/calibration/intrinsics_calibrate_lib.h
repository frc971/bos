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
typedef struct DetectionResult {
  cv::Mat currentCharucoCorners;
  cv::Mat currentCharucoIds;
  std::vector<cv::Point3f> currentObjectPoints;
  std::vector<cv::Point2f> currentImagePoints;

} detection_result_t;

const static int ksquares_x = 2;
const static int ksquares_y = 9;
const static float ksquares_length = 0.025;
const static float kpixel_per_square = 128;
const static float kmarker_length = 1;
const static int kmargin_squares = 0;

cv::aruco::CharucoDetector CreateDetector(
    cv::aruco::Dictionary dictionary, int squares_x = ksquares_x,
    int squares_y = ksquares_y, float squares_length = ksquares_length,
    float pixel_per_square = kpixel_per_square,
    float marker_length = kmarker_length);

json intrisincs_to_json(cv::Mat cameraMatrix, cv::Mat distCoeffs);
cv::Mat GenerateBoard(cv::aruco::CharucoBoard board, int squares_x = ksquares_x,
                      int squares_y = ksquares_y,
                      float pixel_per_square = kpixel_per_square,
                      int margin_squares = kmargin_squares);

detection_result_t DetectCharucoBoard(cv::Mat& frame,
                                      cv::aruco::CharucoDetector detector);

cv::Mat DrawDetectionResult(cv::Mat& frame,
                            detection_result_t detection_result);

double CalibrateCamera(std::vector<detection_result_t> detection_results,
                       cv::Size image_size, cv::Mat& camera_matrix,
                       cv::Mat& distortion_coefficiants);

}  // namespace calibration
