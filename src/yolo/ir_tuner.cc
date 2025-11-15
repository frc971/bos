#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "src/camera/realsense_camera.h"
#include <fstream>

int main() {
  camera::RealSenseCamera rs_camera;
  const int snapshots_per = 5;
  const int step_distance = 3; // inches because I don't have a meter stick :(
  const int num_steps = 20;
  std::vector<std::vector<float>> distances(num_steps, std::vector<float>(snapshots_per));
  for (std::vector<float>& position_results : distances) {
    for (float& distance : position_results) {
      do {
        cv::Mat color_mat;
        cv::Mat depth_mat;
        rs_camera.getFrame(color_mat, depth_mat);
        if (color_mat.empty() || depth_mat.empty()) {
          std::cout << "Couldn't fetch frame properly" << std::endl;
          return 1;
        }
        distance = depth_mat.at<float>(depth_mat.rows/2,depth_mat.cols/2);
        if (distance > 3.5) {
          std::cout << "Detecting too far, please try changing the angle of the camera" << std::endl;
        }
        else if (distance == 0) {
          std::cout << "Didn't get a good detection, might be too close or just try again" << std::endl;
        }
        else {
          std::cout << "Center distance: " << distance << std::endl;
        }
      } while (distance == 0 || distance > 3.5);
    }
    std::cout << "Moving on to the next batch, please go " << step_distance << " inches forward. Waiting for input... " << std::endl;
    int idk_how_to_c;
    std::cin >> idk_how_to_c;
  }
  std::ofstream out("distance_results.txt");
  if (out.is_open()) {
    for (const std::vector<float>& position_results : distances) {
      for (const float& distance : position_results) {
        out << distance << " ";
      }
      out << std::endl;
    }
    out.close();
  }
  for (std::vector<float>& position_results : distances) {
    for (float& distance : position_results) {
       std::cout << distance << " ";
    }
    std::cout << std::endl;
  }
}
