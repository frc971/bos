#include "src/pathing/pathing.h"
#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <wpi/DataLog.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

auto main() -> int {
  // Parse navgrid.json
  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    std::cerr << "Failed to open navgrid.json" << std::endl;
    return 1;
  }
  
  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_W = data["grid"][0].size();
  const int GRID_H = data["grid"].size();
  double nodeSizeMeters = data["nodeSizeMeters"];
  
  // Parse the actual grid data from JSON
  std::vector<std::vector<bool>> gridData(GRID_H, std::vector<bool>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      gridData[y][x] = data["grid"][y][x];
    }
  }
  
  cv::Mat grid = initializeGrid(gridData);

  // Define start and target positions (in grid coordinates)
  int start_x = 10;
  int start_y = 10;
  int target_x = 50;
  int target_y = 10;

  std::cout << "Computing path from (" << start_x << ", " << start_y 
            << ") to (" << target_x << ", " << target_y << ")" << std::endl;

  // Generate smooth spline trajectory
  auto trajectory = createSpline(grid, start_x, start_y, target_x, target_y, nodeSizeMeters);
  
  if (trajectory.empty()) {
    std::cerr << "Failed to create trajectory - no path found!" << std::endl;
    return 1;
  }

  std::cout << "Generated trajectory with " << trajectory.size() << " waypoints" << std::endl;

  // Initialize WPILib logging
  frc::DataLogManager::Start();
  auto& nt = nt::NetworkTableInstance::GetDefault();
  nt.StartServer();

  auto& log = frc::DataLogManager::GetLog();
  
  // Create log entry for robot pose
  wpi::log::StructArrayLogEntry<frc::Pose2d> poseLog(
      log, "/simulator/trajectory");
  wpi::log::DoubleLogEntry xLog(log, "/simulator/pose/x");
  wpi::log::DoubleLogEntry yLog(log, "/simulator/pose/y");
  wpi::log::IntegerLogEntry waypointLog(log, "/simulator/waypoint_index");

  std::cout << "Starting simulation..." << std::endl;

  // Simulate robot movement through trajectory
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& pose = trajectory[i];
    
    // Log the current pose
    xLog.Append(pose.X().value());
    yLog.Append(pose.Y().value());
    waypointLog.Append(static_cast<int64_t>(i));
    
    // Log entire trajectory for visualization
    poseLog.Append(trajectory);

    // Print progress
    if (i % 20 == 0) {
      std::cout << "Waypoint " << i << "/" << trajectory.size() 
                << " - Position: (" << pose.X().value() << ", " 
                << pose.Y().value() << ")" << std::endl;
    }

    // Delay to simulate movement (50ms between waypoints)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::cout << "Simulation complete! Check logs directory for output." << std::endl;

  // Flush logs before exit
  frc::DataLogManager::Stop();

  return 0;
}