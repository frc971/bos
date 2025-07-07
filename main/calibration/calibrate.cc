#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using json = nlohmann::json;

json intrisincs_to_json(cv::Mat cameraMatrix,
                        cv::Mat distCoeffs) { // TODO get index
  assert(cameraMatrix.ptr<float>[8] == 1);

  json output;
  output["fx"] = cameraMatrix.ptr<double>()[0];
  output["cx"] = cameraMatrix.ptr<double>()[2];
  output["fy"] = cameraMatrix.ptr<double>()[4];
  output["cy"] = cameraMatrix.ptr<double>()[5];

  output["k1"] = distCoeffs.ptr<double>()[0];
  output["k2"] = distCoeffs.ptr<double>()[1];
  output["p1"] = distCoeffs.ptr<double>()[2];
  output["p2"] = distCoeffs.ptr<double>()[3];
  output["k3"] = distCoeffs.ptr<double>()[4];

  return output;
}

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  cv::aruco::DetectorParameters detectorParams =
      cv::aruco::DetectorParameters();
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  cv::aruco::CharucoBoard board(cv::Size(5, 7), 0.04, 0.02, dictionary);
  cv::aruco::CharucoParameters charucoParams;
  cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

  std::vector<cv::Mat> allCharucoCorners, allCharucoIds;
  std::vector<std::vector<cv::Point2f>> allImagePoints;
  std::vector<std::vector<cv::Point3f>> allObjectPoints;
  std::vector<cv::Mat> allImages;
  cv::Size imageSize;

  std::vector<cv::String> image_files;
  cv::glob("data/*.jpg", image_files, false);
  std::cout << image_files[0] << std::endl;

  for (const auto &file : image_files) {
    cv::Mat frame = cv::imread(file);

    cv::Mat currentCharucoCorners, currentCharucoIds;
    std::vector<cv::Point3f> currentObjectPoints;
    std::vector<cv::Point2f> currentImagePoints;

    detector.detectBoard(frame, currentCharucoCorners, currentCharucoIds);
    if (currentCharucoCorners.total() > 3) {
      board.matchImagePoints(currentCharucoCorners, currentCharucoIds,
                             currentObjectPoints, currentImagePoints);

      if (currentImagePoints.empty() || currentObjectPoints.empty()) {
        std::cout << "Point matching failed in " << file << std::endl;
        continue;
      }

      std::cout << "Found board in " << file << std::endl;

      allCharucoCorners.push_back(currentCharucoCorners);
      allCharucoIds.push_back(currentCharucoIds);
      allImagePoints.push_back(currentImagePoints);
      allObjectPoints.push_back(currentObjectPoints);
      allImages.push_back(frame);

      imageSize = frame.size();
    } else {
      std::cout << "No board found in " << file << std::endl;
    }
  }

  cv::Mat cameraMatrix, distCoeffs;

  std::cout << "Calculating intrinsics..." << std::endl;
  double repError =
      calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix,
                      distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(),
                      cv::noArray(), cv::noArray());
  std::cout << "Done! Error: " << repError << std::endl;

  std::ofstream file("intrinsics.json");
  json intrinsics = intrisincs_to_json(cameraMatrix, distCoeffs);
  file << intrinsics.dump(4);
  std::cout << "Intrinsics: " << intrinsics.dump(4);
  file.close();
  return 0;
}
