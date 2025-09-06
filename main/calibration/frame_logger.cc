#include <functional>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <atomic>
#include "main/camera/camera.h"
#include "main/camera/streamer.h"


void read_camera(Camera::Streamer streamer, Camera::Camera camera, std::atomic<bool>& log_image, std::string data_folder){
  cv::Mat frame;
  int image_idx = 0;
  std::ostringstream filename;
  while (true){
    camera.getFrame(frame);
    streamer.writeFrame(frame);
    if (log_image.load()){
      filename << data_folder << std::setfill('0') << std::setw(4) << image_idx << ".jpg";
      cv::imwrite(filename.str(), frame);
      log_image.store(false);
    }
  }
}

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are logging?\n";
  int camera_id;
  std::cin >> camera_id;

  Camera::CameraInfo camera_info;

  switch (camera_id){
  case 0:
    camera_info = Camera::CAMERAS.gstreamer1_30fps;
    break;
  case 1:
    camera_info = Camera::CAMERAS.gstreamer1_30fps;
    break;
  default:
    std::cout << "Invalid ID! Only 0 or 1" << std::endl;
    return 0;
  }

  std::string data_folder = "data/camera_" + std::to_string(camera_id) + "/";
  if (std::filesystem::create_directory(data_folder)){
    std::cout << "data folder created successfully!\n";
  }
  else {
    std::cout << "do you want to delete the existing photos? (yes/no)\n";
    std::string delete_existing_photos;
    std::cin >> delete_existing_photos;
    if (delete_existing_photos == "yes"){
      std::filesystem::remove_all(data_folder);
      std::filesystem::create_directory(data_folder);
    }
  }

  Camera::Camera camera(Camera::CAMERAS.gstreamer1_30fps);
  Camera::Streamer streamer(4097, true);
  std::atomic<bool> log_image(false);

  cv::Mat frame;
  int image_idx = 0;

  std::cout << "Camera opened successfully. Press 'c' to capture, 'q' to quit."
            << std::endl;

  std::thread read_camera_thread(read_camera, std::move(streamer), std::move(camera), std::ref(log_image), data_folder);

  while (true) {
    char key;
    std::cin >> key;
    switch (key){
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
