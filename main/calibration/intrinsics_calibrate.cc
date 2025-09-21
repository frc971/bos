#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include "main/calibration/intrinsics_calibrate_lib.h"
#include "main/camera/imx296_camera.h"
#include "main/camera/streamer.h"

using json = nlohmann::json;

void CaptureFrames(
    cv::aruco::CharucoDetector detector, camera::IMX296Camera camera,
    camera::Streamer& streamer,
    std::vector<Calibration::detection_result_t>& detection_results,
    std::atomic<bool>& capture_frames_thread) {
  cv::Mat frame;
  int frame_count = 0;
  while (true) {
    camera.getFrame(frame);
    frame_count++;
    if (frame_count % 10 == 0) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
      Calibration::detection_result_t detection_result =
          Calibration::DetectCharucoBoard(frame, detector);

      cv::Mat annotated_frame = DrawDetectionResult(frame, detection_result);
      for (Calibration::detection_result_t detection_result :
           detection_results) {
        annotated_frame =
            Calibration::DrawDetectionResult(annotated_frame, detection_result);
      }
      streamer.WriteFrame(annotated_frame);

      detection_results.push_back(detection_result);
    }
    if (!capture_frames_thread.load()) {
      return;
    }
  }
}

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are logging?\n";
  int camera_id;
  std::cin >> camera_id;

  camera::CameraInfo camera_info;
  switch (camera_id) {
    case 0:
      camera_info = camera::CAMERAS.gstreamer1_30fps;
      break;
    case 1:
      camera_info = camera::CAMERAS.gstreamer2_30fps;
      break;
    default:
      std::cout << "Invalid ID! Only 0 or 1" << std::endl;
      return 0;
  }

  camera::Streamer streamer(4971, true);
  camera::IMX296Camera camera(camera_info);

  cv::Mat frame;
  camera.getFrame(frame);
  cv::Size frame_size = frame.size();

  cv::aruco::CharucoDetector detector = Calibration::CreateDetector(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250));
  cv::Mat board_image = Calibration::GenerateBoard(detector.getBoard());
  cv::imwrite("calibration_board.png", board_image);

  std::atomic<bool> capture_frames(true);
  std::vector<Calibration::detection_result_t> detection_results;
  std::thread capture_frames_thread(
      CaptureFrames, detector, camera, std::ref(streamer),
      std::ref(detection_results), std::ref(capture_frames));
  std::cout << "Double enter to finish capturing frames" << std::endl;
  std::cin.get();
  std::cin.get();
  capture_frames.store(false);
  capture_frames_thread.join();
  std::cout << "Calibrating cameras" << std::endl;

  cv::Mat cameraMatrix, distCoeffs;
  Calibration::CalibrateCamera(detection_results, frame_size, cameraMatrix,
                               distCoeffs);

  std::ofstream file(camera_info.intrinsics_path);
  json intrinsics = Calibration::intrisincs_to_json(cameraMatrix, distCoeffs);
  file << intrinsics.dump(4);
  std::cout << "Intrinsics: " << intrinsics.dump(4);
  file.close();
}
