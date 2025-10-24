#include "intrinsics_calibrate_lib.h"
#include <opencv2/objdetect/charuco_detector.hpp>
#include "main/camera/imx296_camera.h"
#include "main/camera/streamer.h"

namespace calibration {

json intrisincs_to_json(cv::Mat cameraMatrix,
                        cv::Mat distCoeffs) {  // TODO get index

  json output;
  output["fx"] = cameraMatrix.ptr<double>()[0];
  output["cx"] = cameraMatrix.ptr<double>()[2];
  output["fy"] = cameraMatrix.ptr<double>()[4];
  output["cy"] = cameraMatrix.ptr<double>()[5];

  output["k1"] = distCoeffs.ptr<double>()[0];
  output["k2"] = distCoeffs.ptr<double>()[1];
  output["p1"] = distCoeffs.ptr<double>()[2];
  output["p2"] = distCoeffs.ptr<double>()[3];
  output["k3"] = distCoeffs.ptr<double>()[4];

  return output;
}

cv::aruco::CharucoDetector CreateDetector(cv::aruco::Dictionary dictionary,
                                          int squares_x, int squares_y,
                                          float squares_length,
                                          float pixel_per_square,
                                          float marker_length) {

  cv::aruco::CharucoBoard board(cv::Size(squares_x, squares_y), squares_length,
                                marker_length, dictionary);

  cv::aruco::DetectorParameters detectorParams =
      cv::aruco::DetectorParameters();
  cv::aruco::CharucoParameters charucoParams;
  cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);
  return detector;
}

cv::Mat GenerateBoard(cv::aruco::CharucoBoard board, int squares_x,
                      int squares_y, float pixel_per_square,
                      int margin_squares) {

  cv::Mat board_image;
  cv::Size image_size;
  image_size.width = (squares_x + margin_squares) * pixel_per_square;
  image_size.height = (squares_y + margin_squares) * pixel_per_square;
  board.generateImage(image_size, board_image,
                      margin_squares * pixel_per_square, 1);
  return board_image;
}

detection_result_t DetectCharucoBoard(cv::Mat& frame,
                                      cv::aruco::CharucoDetector detector) {

  detection_result_t detection_result;

  detector.detectBoard(frame, detection_result.currentCharucoCorners,
                       detection_result.currentCharucoIds);
  if (detection_result.currentCharucoCorners.total() > 3) {
    detector.getBoard().matchImagePoints(detection_result.currentCharucoCorners,
                                         detection_result.currentCharucoIds,
                                         detection_result.currentObjectPoints,
                                         detection_result.currentImagePoints);
  }
  return detection_result;
}

cv::Mat DrawDetectionResult(cv::Mat& frame,
                            detection_result_t detection_result) {
  cv::Mat result;
  frame.copyTo(result);

  if (detection_result.currentCharucoCorners.total() > 3) {
    cv::aruco::drawDetectedCornersCharuco(
        result, detection_result.currentCharucoCorners,
        detection_result.currentCharucoIds);
  }
  return result;
}

double CalibrateCamera(std::vector<detection_result_t> detection_results,
                       cv::Size image_size, cv::Mat& camera_matrix,
                       cv::Mat& distortion_coefficiants) {

  std::vector<cv::Mat> allCharucoCorners, allCharucoIds;
  std::vector<std::vector<cv::Point2f>> allImagePoints;
  std::vector<std::vector<cv::Point3f>> allObjectPoints;

  for (detection_result_t detection_result : detection_results) {
    if (detection_result.currentCharucoCorners.total() > 3) {
      if (!detection_result.currentImagePoints.empty() &&
          !detection_result.currentObjectPoints.empty()) {
        allCharucoCorners.push_back(detection_result.currentCharucoCorners);
        allCharucoIds.push_back(detection_result.currentCharucoIds);
        allImagePoints.push_back(detection_result.currentImagePoints);
        allObjectPoints.push_back(detection_result.currentObjectPoints);
      }
    }
  }

  std::cout << "Got " << allCharucoCorners.size() << " detections" << std::endl;

  return cv::calibrateCamera(allObjectPoints, allImagePoints, image_size,
                             camera_matrix, distortion_coefficiants,
                             cv::noArray(), cv::noArray(), cv::noArray(),
                             cv::noArray(), cv::noArray());
}

}  // namespace calibration
