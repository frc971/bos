#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "main/camera/imx296_camera.h"
#include "main/localization/tag_estimator.h"

using json = nlohmann::json;

int main() {
  std::cout << "What is the id of the camera?\n";
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

  json intrinsics;
  std::ifstream intrinsics_file(camera_info.intrinsics_path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: "
              << camera_info.intrinsics_path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }

  std::cout << "What tag are we calibrating against?\n";
  int tag_id;
  std::cin >> tag_id;

  camera::IMX296Camera camera(camera_info);
  cv::Mat frame;

  localization::TagEstimator estimator(intrinsics, nullptr);

  localization::pose3d_t average_position{{0, 0, 0}, {0, 0, 0}};

  int estimate_count = 0;
  for (int i = 0; i < 24; i++) {
    camera.getFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        estimator.GetRawPositionEstimates(frame);
    estimate_count += estimates.size();
    for (localization::tag_detection_t& estimate : estimates) {
      if (estimate.tag_id == tag_id) {
        average_position.rotation.x += estimate.rotation.x;
        average_position.rotation.y += estimate.rotation.y;
        average_position.rotation.z += estimate.rotation.z;

        average_position.translation.x += estimate.translation.x;
        average_position.translation.y += estimate.translation.y;
        average_position.translation.z += estimate.translation.z;
      }
    }
  }
  std::cout << "Estimate count: " << estimate_count << std::endl;
  average_position.rotation.x /= estimate_count;
  average_position.rotation.y /= estimate_count;
  average_position.rotation.z /= estimate_count;

  average_position.translation.x /= estimate_count;
  average_position.translation.y /= estimate_count;
  average_position.translation.z /= estimate_count;

  std::cout << "Estimated position: " << std::endl;

  localization::tag_detection_t true_position;
  std::cout << "True position x (meters)";
  std::cin >> true_position.translation.x;

  std::cout << "True position y (meters)";
  std::cin >> true_position.translation.y;

  std::cout << "True position z (meters)";
  std::cin >> true_position.translation.z;

  std::cout << "True rotation x (meters)";
  std::cin >> true_position.rotation.x;

  std::cout << "True rotation y (meters)";
  std::cin >> true_position.rotation.y;

  std::cout << "True rotation z (meters)";
  std::cin >> true_position.rotation.z;

  localization::tag_detection_t extrinsics;
  extrinsics.translation.x =
      average_position.translation.x - true_position.translation.x;
  extrinsics.translation.y =
      average_position.translation.y - true_position.translation.y;
  extrinsics.translation.z =
      average_position.translation.z - true_position.translation.z;

  extrinsics.rotation.x =
      average_position.rotation.x - true_position.rotation.x;
  extrinsics.rotation.y =
      average_position.rotation.y - true_position.rotation.y;
  extrinsics.rotation.z =
      average_position.rotation.z - true_position.rotation.z;

  std::ofstream file(camera_info.extrinsics_path);
  json extrinsics_json = localization::ExtrinsicsToJson(extrinsics);
  file << extrinsics_json.dump(4);
  std::cout << "Extrinsics: \n" << intrinsics.dump(4);
  file.close();
}
