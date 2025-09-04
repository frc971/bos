#include <iomanip>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sstream>

int main(int argc, char* argv[]) {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  bool gui_mode = false;
  if (argc >= 2 && std::string(argv[1]) == "--gui"){
    gui_mode = true;
    std::cout << "gui mode on\n";
  } else{
    std::cout << "gui mode off (turn on with --gui)\n";
  }

  std::cout << "What is the id of the camera we are logging?\n";
  int camera_id;
  std::cin >> camera_id;

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


  std::string pipeline =
      "nvarguscamerasrc sensor-id=" + std::to_string(camera_id) + 
      " aelock=true exposuretimerange=\"100000 " + 
      "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! " + 
      "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, " + 
      "format=NV12 ! " + 
      "nvvidconv ! " + 
      "video/x-raw, format=BGRx ! " + 
      "appsink";

  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open camera" << std::endl;
    return -1;
  }

  cv::Mat frame;
  int image_idx = 0;

  std::cout << "Camera opened successfully. Press 'c' to capture, 'q' to quit."
            << std::endl;

  while (true) {
    cap >> frame;
    if (frame.empty()) {
      std::cerr << "Error: Could not read frame" << std::endl;
      break;
    }

    char key;
    if (gui_mode){
      cv::imshow("Camera Feed", frame);
      key = cv::waitKey(1) & 0xFF;
    }
    else {
      std::cin >> key;
    }
    std::cout << key;

    cv::imwrite("data/server_img.jpg", frame);

    if (key == 'q') {
      break;
    } else if (key == 'c') {
      std::ostringstream filename;
      filename << data_folder << std::setfill('0') << std::setw(4) << image_idx << ".jpg";

      if (cv::imwrite(filename.str(), frame)) {
        std::cout << "Saved: " << filename.str() << std::endl;
        image_idx++;
      } else {
        std::cerr << "Error: Could not save image " << filename.str()
                  << std::endl;
      }
    }
  }

  cap.release();
  cv::destroyAllWindows();
  return 0;
}
