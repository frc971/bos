#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::string pipeline =
      "nvarguscamerasrc sensor-id=0 aelock=true exposuretimerange=\"100000 "
      "200000\" gainrange=\"1 15\" ispdigitalgainrange=\"1 1\" ! "
      "video/x-raw(memory:NVMM), width=1456, height=1088, framerate=30/1, "
      "format=NV12 ! "
      "nvvidconv ! "
      "video/x-raw, format=BGRx ! "
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

    cv::imshow("Camera Feed", frame);

    char key = cv::waitKey(1) & 0xFF;

    if (key == 'q') {
      break;
    } else if (key == 'c') {
      std::ostringstream filename;
      filename << "data/img" << std::setfill('0') << std::setw(4) << image_idx
               << ".jpg";

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
