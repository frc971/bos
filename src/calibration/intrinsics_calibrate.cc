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
#include "src/calibration/intrinsics_calibrate_lib.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

using json = nlohmann::json;

void CaptureFrames(
    const cv::aruco::CharucoDetector& detector, camera::ICamera& camera,
    camera::CscoreStreamer streamer,
    std::vector<calibration::detection_result_t>& detection_results,
    std::atomic<bool>& capture_frames_thread, std::atomic<bool>& log_image) {
  cv::Mat frame;
  int frame_count = 0;
  while (true) {
    camera.GetFrame(frame);
    frame_count++;
    if (frame_count % 3 == 0) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
      calibration::detection_result_t detection_result =
          calibration::DetectCharucoBoard(frame, detector);

      cv::Mat annotated_frame = DrawDetectionResult(frame, detection_result);
      for (const calibration::detection_result_t& detection_result :
           detection_results) {
        annotated_frame =
            calibration::DrawDetectionResult(annotated_frame, detection_result);
      }
      streamer.WriteFrame(annotated_frame);

      if (log_image.load()) {
        detection_results.push_back(detection_result);
        log_image.store(false);
      }
    }
    if (!capture_frames_thread.load()) {
      return;
    }
  }
}

auto main() -> int {
  camera::CscoreStreamer streamer("intrinsics_calibrate", 4971, 30, 1080, 1080,
                                  true);

  camera::Camera config = camera::SelectCameraConfig();
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);

  cv::Mat frame;
  camera->GetFrame(frame);
  cv::Size frame_size = frame.size();

  cv::aruco::CharucoDetector detector = calibration::CreateDetector(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250));
  cv::Mat board_image = calibration::GenerateBoard(detector.getBoard());
  cv::imwrite("calibration_board.png", board_image);

  std::atomic<bool> capture_frames(true);
  std::atomic<bool> log_image(false);

  std::vector<calibration::detection_result_t> detection_results;
  std::thread capture_frames_thread(
      CaptureFrames, std::ref(detector), std::ref(*camera), streamer,
      std::ref(detection_results), std::ref(capture_frames),
      std::ref(log_image));

  bool run = true;
  while (run) {
    char key;
    std::cin >> key;
    switch (key) {
      case 'q':
        run = false;
      case 'c':
        log_image.store(true);
        break;
      default:
        std::cout << "Received invalid key!\n";
    }
  }

  capture_frames.store(false);
  capture_frames_thread.join();
  std::cout << "Calibrating cameras" << std::endl;

  cv::Mat cameraMatrix, distCoeffs;
  calibration::CalibrateCamera(detection_results, frame_size, cameraMatrix,
                               distCoeffs);

  std::ofstream file("intrinsics.json");
  json intrinsics = calibration::intrisincs_to_json(cameraMatrix, distCoeffs);
  file << intrinsics.dump(4);
  std::cout << "Intrinsics: \n" << std::endl << intrinsics.dump(4) << std::endl;
  file.close();
}
