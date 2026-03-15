#pragma once
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_solver.h"
#include "src/utils/pch.h"

namespace localization {
// Runs localization given a camerasource, detector and solver.
// Example:
// std::thread front_right_thread(
//     localization::run_localization, std::ref(front_right_camera),
//     std::make_unique<localization::GPUAprilTagDetector>(
//         front_right_camera.GetFrame().cols, front_right_camera.GetFrame().rows,
//         utils::read_intrinsics(
//             camera::camera_constants[camera::CameraConstant::TURRET_BOT_FRONT_RIGHT]
//                 .intrinsics_path)),
//     std::make_unique<localization::SquareSolver>(
//         camera::CameraConstant::TURRET_BOT_FRONT_RIGHT),
//     camera::camera_constants[camera::CameraConstant::TURRET_BOT_FRONT_RIGHT]
//         .extrinsics_path,
//     4971, false);
void RunLocalization(camera::CameraSource& source,
                     std::unique_ptr<localization::IAprilTagDetector> detector,
                     std::unique_ptr<localization::IPositionSolver> solver,
                     const std::string& extrinsics,
                     std::optional<uint> port = std::nullopt,
                     bool verbose = false);
void RunJointSolve(
    std::vector<std::pair<camera::CameraConstant,
                          std::unique_ptr<camera::CameraSource>>>&
        camera_sources,
    uint port, bool square_solve_start, bool verbose);
auto GetSquareSolveEstimates(
    std::vector<std::pair<camera::CameraConstant,
                          std::unique_ptr<camera::CameraSource>>>&
        camera_sources,
    std::vector<localization::OpenCVAprilTagDetector>& detectors)
    -> position_estimate_t;
auto GetSquareSolveEstimates(
    const std::map<camera::CameraConstant, std::vector<tag_detection_t>>&
        associated_frames) -> position_estimate_t;
}  // namespace localization
