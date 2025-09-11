#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace Calibration {

using json = nlohmann::json;
typedef struct DetectionResult {
  cv::Mat currentCharucoCorners;
  cv::Mat currentCharucoIds;
  std::vector<cv::Point3f> currentObjectPoints;
  std::vector<cv::Point2f> currentImagePoints;

} detection_result_t;

cv::aruco::CharucoDetector CreateDetector(cv::aruco::Dictionary dictionary,
                                          int squares_x = 5, int squares_y = 7,
                                          float squares_length = 0.04,
                                          float pixel_per_square = 128,
                                          float marker_length = 0.02);

json intrisincs_to_json(cv::Mat cameraMatrix, cv::Mat distCoeffs);
cv::Mat GenerateBoard(cv::aruco::CharucoBoard board, int squares_x = 5,
                      int squares_y = 7, float pixel_per_square = 128,
                      int margin_squares = 0);

detection_result_t DetectCharucoBoard(cv::Mat& frame,
                                      cv::aruco::CharucoDetector detector);

cv::Mat DrawDetectionResult(cv::Mat& frame,
                            detection_result_t detection_result);

double CalibrateCamera(std::vector<detection_result_t> detection_results,
                       cv::Size image_size, cv::Mat& camera_matrix,
                       cv::Mat& distortion_coefficiants);

}  // namespace Calibration
