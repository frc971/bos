#include <opencv2/opencv.hpp>
#include "src/localization/square_solver.h"
#include "src/camera/camera_constants.h"

constexpr double ktag_size = 0.1651;
const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout("/bos/constants/2026-rebuilt-andymark.json");

// Generate tag corners in tag's local coordinate system
auto GetTagCorners(double tag_size) -> std::vector<cv::Point3f> {
  double half_size = tag_size / 2.0;
  return {
      cv::Point3f(-half_size, half_size, 0.0f),   // top-left
      cv::Point3f(half_size, half_size, 0.0f),    // top-right
      cv::Point3f(half_size, -half_size, 0.0f),   // bottom-right
      cv::Point3f(-half_size, -half_size, 0.0f)   // bottom-left
  };
}

auto main(int argc, char* argv[]) -> int {
  auto config = camera::Camera::STOVETOP_BOT_FRONT_RIGHT;
  
  std::vector<cv::Point3f> tag_corners = GetTagCorners(ktag_size);
  
  localization::SquareSolver solver(
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path,
      kapriltag_layout,
      tag_corners);

  // Create multiple detections of the same tag for testing
  std::vector<localization::tag_detection_t> tag_detections;
  
  localization::tag_detection_t detection1;
  detection1.tag_id = 1; 
  detection1.timestamp = 0.0;
  detection1.corners = {
      cv::Point2f(100.0f, 100.0f),
      cv::Point2f(200.0f, 100.0f),
      cv::Point2f(200.0f, 200.0f),
      cv::Point2f(100.0f, 200.0f)
  };
  tag_detections.push_back(detection1);
  
  localization::tag_detection_t detection2;
  detection2.tag_id = 1;
  detection2.timestamp = 0.0;
  detection2.corners = {
      cv::Point2f(100.0f, 100.0f),
      cv::Point2f(200.0f, 100.0f),
      cv::Point2f(200.0f, 200.0f),
      cv::Point2f(100.0f, 200.0f)
  };
  tag_detections.push_back(detection2);
  
  localization::tag_detection_t detection3;
  detection3.tag_id = 1;
  detection3.timestamp = 0.0;
  detection3.corners = {
      cv::Point2f(100.0f, 100.0f),
      cv::Point2f(200.0f, 100.0f),
      cv::Point2f(200.0f, 200.0f),
      cv::Point2f(100.0f, 200.0f)
  };
  tag_detections.push_back(detection3);

  LOG(INFO) << "Testing SquareSolver with " << tag_detections.size() 
            << " detections of the same tag (ID " << detection1.tag_id << ")";
  
  LOG(INFO) << "Individual tag solving";
  std::vector<localization::position_estimate_t> individual_estimates =
      solver.EstimatePosition(tag_detections);

  for (size_t i = 0; i < individual_estimates.size(); ++i) {
    LOG(INFO) << "Detection " << i+1 << " (Tag " << tag_detections[i].tag_id 
              << ") - Position: " << individual_estimates[i];
  }
  
  // Calculate average of individual estimates
  if (!individual_estimates.empty()) {
    double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
    double avg_roll = 0.0, avg_pitch = 0.0, avg_yaw = 0.0;
    for (const auto& est : individual_estimates) {
      avg_x += est.pose.X().value();
      avg_y += est.pose.Y().value();
      avg_z += est.pose.Z().value();
      avg_roll += est.pose.Rotation().X().value();
      avg_pitch += est.pose.Rotation().Y().value();
      avg_yaw += est.pose.Rotation().Z().value();
    }
    avg_x /= individual_estimates.size();
    avg_y /= individual_estimates.size();
    avg_z /= individual_estimates.size();
    avg_roll /= individual_estimates.size();
    avg_pitch /= individual_estimates.size();
    avg_yaw /= individual_estimates.size();
    
    LOG(INFO) << "Average of individual estimates:";
    LOG(INFO) << "  Position: (" << avg_x << " m, " << avg_y << " m, " << avg_z << " m)";
    LOG(INFO) << "  Rotation: (" << avg_roll*180/M_PI << " deg, " 
              << avg_pitch*180/M_PI << " deg, " << avg_yaw*180/M_PI << " deg)";
  }

  LOG(INFO) << "\nJoint Tag Solving";
  auto joint_estimate = solver.EstimatePositionJoint(tag_detections);
  
  if (joint_estimate.has_value()) {
    LOG(INFO) << "Joint estimate: " << joint_estimate.value();
  } else {
    LOG(WARNING) << "Joint solving failed!";
  }

  if (joint_estimate.has_value() && !individual_estimates.empty()) {
    double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
    for (const auto& est : individual_estimates) {
      avg_x += est.pose.X().value();
      avg_y += est.pose.Y().value();
      avg_z += est.pose.Z().value();
    }
    avg_x /= individual_estimates.size();
    avg_y /= individual_estimates.size();
    avg_z /= individual_estimates.size();
    
    double diff_x = joint_estimate->pose.X().value() - avg_x;
    double diff_y = joint_estimate->pose.Y().value() - avg_y;
    double diff_z = joint_estimate->pose.Z().value() - avg_z;
    double diff_magnitude = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
    
    LOG(INFO) << "Position difference (joint vs avg individual):";
    LOG(INFO) << "  Delta: (" << diff_x << " m, " << diff_y << " m, " << diff_z << " m)";
    LOG(INFO) << "  Magnitude: " << diff_magnitude << " m";
    
    if (diff_magnitude < 0.001) {
      LOG(INFO) << "PASS: Results are essentially identical (tested <1mm difference)";
    } else {
      LOG(WARNING) << "FAIL";
    }
  }
  LOG(INFO) << "\n\nTest complete!";

  return 0;
}