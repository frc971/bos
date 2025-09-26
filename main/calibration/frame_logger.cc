#include <atomic>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include "main/camera/cscore_streamer.h"
#include "main/camera/imx296_camera.h"

const int k_port = 5200;

void read_camera(camera::CscoreStreamer streamer, camera::IMX296Camera camera,
                 std::atomic<bool>& log_image, std::string data_folder) {
  cv::Mat frame;
  int image_idx = 0;
  while (true) {
    camera.getFrame(frame);
    streamer.WriteFrame(frame);
    if (log_image.load()) {
      std::ostringstream filename;
      filename << data_folder << std::setfill('0') << std::setw(4) << image_idx
               << ".jpg";
      std::cout << "writing frame to " << filename.str() << "\n";
      cv::imwrite(filename.str(), frame);
      log_image.store(false);
      image_idx++;
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
      camera_info = camera::gstreamer1_30fps;
      break;
    case 1:
      camera_info = camera::gstreamer2_30fps;
      break;
    default:
      std::cout << "Invalid ID! Only 0 or 1" << std::endl;
      return 0;
  }

  std::string data_folder = "data/camera_" + std::to_string(camera_id) + "/";
  if (std::filesystem::create_directory(data_folder)) {
    std::cout << "data folder created successfully!\n";
  } else {
    std::cout << "do you want to delete the existing photos? (yes/no)\n";
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
  camera::IMX296Camera camera(camera_info);
  std::atomic<bool> log_image(false);

  cv::Mat frame;

  std::cout << "Camera opened successfully. Press 'c' to capture, 'q' to quit."
            << std::endl;

  std::thread read_camera_thread(read_camera, std::move(streamer),
                                 std::move(camera), std::ref(log_image),
                                 data_folder);

  while (true) {
    char key;
    std::cin >> key;
    switch (key) {
      case 'q':
        return 0;
      case 'c':
        log_image.store(true);
        continue;
      default:
        std::cout << "Received invalid key!\n";
    }
  }

  cv::destroyAllWindows();
  return 0;
}
