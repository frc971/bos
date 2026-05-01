#include <algorithm>
#include <iostream>
#include <string>
#include "nlohmann/json.hpp"
#include "src/pathing/pathfinding.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

auto main(int argc, char* argv[]) -> int {

  std::vector<double> result = pathing::KnotVector(15, 3);
  for (double res : result) {
    std::cout << res << std::endl;
  }
  std::flush(std::cout);
  std::vector<std::string> argstrings(&argv[1], &argv[argc]);

  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open navgrid.json" << std::endl;
    return 0;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_H = data["grid"].size();
  const int GRID_W = data["grid"][0].size();
  double nodeSizeMeters = data["nodeSizeMeters"];
  LOG(INFO) << nodeSizeMeters;

  std::vector<std::vector<pathing::Node>> field(
      GRID_H, std::vector<pathing::Node>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      field[y][x].x = x;
      field[y][x].y = y;
      field[y][x].obstacle = !data["grid"][y][x];
    }
  }

  cv::namedWindow("Pathing", cv::WINDOW_AUTOSIZE);
  cv::Mat mat(GRID_W, GRID_H, CV_8UC3);

  while (true) {

    for (int y = 0; y < GRID_H; ++y) {
      for (int x = 0; x < GRID_W; ++x) {
        pathing::Node n = {.x = (uint)x, .y = (uint)y};
        n.obstacle = data["grid"][y][x];
        n.readable = '-';
        if (n.obstacle) {
          n.readable = '#';
        }
        field[y][x] = n;
      }
    }

    double vision_estimate_x = 2.21643;
    double vision_estimate_y = 5.21322;

    pathing::Point start = {.x = (uint)(vision_estimate_x / nodeSizeMeters),
                            .y = (uint)(vision_estimate_y / nodeSizeMeters)};
    pathing::Point end = {.x = 22, .y = 14};
    std::vector<pathing::Node> result = pathing::BFS(field, start, end);
    if (std::count(argstrings.begin(), argstrings.end(), "-v")) {
      std::string readable = "";
      for (pathing::Node n : result) {
        std::string intermediate = "";
        intermediate.append("(");
        intermediate.append(std::to_string(n.x));
        intermediate.append(", ");
        intermediate.append(std::to_string(n.y));
        intermediate.append("), ");
        readable.append(intermediate);
        field[n.y][n.x].readable = '*';
      }
      std::cout << readable << std::endl;

      for (int y = 0; y < GRID_H; ++y) {
        for (int x = 0; x < GRID_W; ++x) {
          std::cout << field[y][x].readable;
        }
        std::cout << std::endl;
      }
    }
    for (int y = 0; y < 15; y++) {
      for (int x = 0; x < 15; x++) {
        mat.at<cv::Vec3b>(y, x) = field[y][x].obstacle
                                      ? cv::Vec3b(0, 0, 0)
                                      : cv::Vec3b(255, 255, 255);

        if (field[y][x].path) {
          mat.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
        }
      }
    }

    cv::Mat display;

    /* Uncomment the following lines to display the grid, else it will just be displayed in console.*/

    // cv::resize(mat, display, cv::Size(450, 450), 0, 0, cv::INTER_NEAREST);
    // cv::imshow("Grid", display);
    // cv::waitKey();
  }
}