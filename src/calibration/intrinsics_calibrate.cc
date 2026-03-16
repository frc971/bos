#include <utility>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/calibration/intrinsics_calibrate_lib.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT
ABSL_FLAG(std::optional<int>, port, std::nullopt, "");                 //NOLINT

using json = nlohmann::json;

void CaptureFrames(
    const cv::aruco::CharucoDetector& detector, camera::CameraSource& source,
    camera::CscoreStreamer streamer,
    std::vector<calibration::detection_result_t>& detection_results,
    std::atomic<bool>& capture_frames_thread, std::atomic<bool>& log_image) {
  cv::Mat frame;
  cv::Mat bgr_frame;
  int frame_count = 0;
  while (true) {
    frame = source.GetFrame();
    frame_count++;
    if (frame_count % 1 == 0) {
      cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
      calibration::detection_result_t detection_result =
          calibration::DetectCharucoBoard(bgr_frame, detector);

      cv::Mat annotated_frame =
          DrawDetectionResult(bgr_frame, detection_result);
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

auto WriteIntrinsicToFile(cv::Mat camera_matrix, cv::Mat dist_coeffs,
                          const std::string& path, bool print = false) {
  std::ofstream intrinsics_file(path);
  json intrinsics = calibration::IntrinsicsToJson(std::move(camera_matrix),
                                                  std::move(dist_coeffs));
  intrinsics_file << intrinsics.dump(4);
  if (print) {
    std::cout << "Intrinsics: \n"
              << std::endl
              << intrinsics.dump(4) << std::endl;
  }
  intrinsics_file.close();
}

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  camera::CameraSource source = camera::SelectCamera(
      "camera", absl::GetFlag(FLAGS_camera_name),
      camera::GetCameraConstants());
  camera::CscoreStreamer streamer("intrinsics_calibrate",
                                  absl::GetFlag(FLAGS_port).value_or(5801), 30,
                                  source.GetFrame());

  cv::Mat frame = source.GetFrame();
  cv::Size frame_size = frame.size();

  cv::aruco::CharucoDetector detector = calibration::CreateDetector(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250));
  cv::Mat board_image = calibration::GenerateBoard(detector.getBoard());
  cv::imwrite("calibration_board.png", board_image);

  std::atomic<bool> capture_frames(true);
  std::atomic<bool> log_image(false);

  std::vector<calibration::detection_result_t> detection_results;
  std::thread capture_frames_thread(
      CaptureFrames, std::ref(detector), std::ref(source), streamer,
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

  WriteIntrinsicToFile(cameraMatrix, distCoeffs, "intrinsics.json", true);
}
