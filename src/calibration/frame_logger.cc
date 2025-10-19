#include "include/pch.h"

#include <atomic>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/imx296_camera.h"
#include "src/camera/select_camera.h"

const int k_port = 4971;

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are logging?\\n";
  int camera_id;
  std::cin >> camera_id;

  std::string data_folder = "data/camera_" + std::to_string(camera_id) + "/";
  if (std::filesystem::create_directory(data_folder)) {
    std::cout << "data folder created successfully!\\n";
  } else {
    std::cout << "do you want to delete the existing photos? (yes/no)\\n";
    std::string delete_existing_photos;
    std::cin >> delete_existing_photos;
    if (delete_existing_photos == "yes") {
      std::filesystem::remove_all(data_folder);
      std::filesystem::create_directory(data_folder);
    }
  }

  std::cout << "Port number: " << k_port << std::endl;

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("frame_logger", 4971, 30));
  camera::CVCamera camera = camera::SelectCamera();
  std::atomic<bool> log_image(false);

  cv::Mat frame;

  std::cout << "Camera opened successfully. Press 'c' to capture, 'q' to quit."
            << std::endl;

  while (true) {
    cv::Mat frame;
    int image_idx = 0;
    while (true) {
      camera.GetFrame(frame);
      streamer.WriteFrame(frame);
      std::ostringstream filename;
      filename << data_folder << std::setfill('0') << std::setw(4) << image_idx
               << ".jpg";
      std::cout << "writing frame to " << filename.str() << "\n";
      cv::imwrite(filename.str(), frame);
      log_image.store(false);
      image_idx++;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
