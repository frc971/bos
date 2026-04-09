#include <gtest/gtest.h>
#include <fstream>
#include <vector>
#include "nlohmann/json.hpp"
#include "src/pathing/pathfinding.h"

TEST(PathingTest, BFSPath) {  //NOLINT
  std::vector<pathing::Node> expected = {
      {.x = 8, .y = 16},  {.x = 9, .y = 15},  {.x = 10, .y = 16},
      {.x = 11, .y = 17}, {.x = 12, .y = 18}, {.x = 13, .y = 18},
      {.x = 14, .y = 18}, {.x = 15, .y = 18}, {.x = 16, .y = 18},
      {.x = 17, .y = 18}, {.x = 18, .y = 18}, {.x = 19, .y = 17},
      {.x = 20, .y = 16}, {.x = 21, .y = 15}, {.x = 22, .y = 14}};

  std::ifstream file("/root/bos/constants/navgrid.json");

  nlohmann::json json = nlohmann::json::parse(file);
  file.close();

  const int GRID_H = json["grid"].size();
  const int GRID_W = json["grid"][0].size();

  std::vector<std::vector<pathing::Node>> field(
      GRID_H, std::vector<pathing::Node>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      field[y][x].x = x;
      field[y][x].y = y;
      field[y][x].obstacle = json["grid"][y][x];
    }
  }

  pathing::Node& startn = field[17][7];
  pathing::Node& endn = field[14][22];
  pathing::Point start = {.x = startn.x, .y = startn.y};
  pathing::Point end = {.x = endn.x, .y = endn.y};

  std::vector<pathing::Node> res = pathing::BFS(field, start, end);

  EXPECT_EQ(expected, res);
}