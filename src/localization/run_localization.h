#pragma once
#include "src/camera/camera_source.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/multi_camera_detector.h"
#include "src/localization/position_sender.h"
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
//             camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
//                 .intrinsics_path)),
//     std::make_unique<localization::SquareSolver>(
//         camera::Camera::TURRET_BOT_FRONT_RIGHT),
//     camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
//         .extrinsics_path,
//     5801, false);
void RunLocalization(
    std::unique_ptr<camera::CameraSource> source,
    std::unique_ptr<localization::IAprilTagDetector> detector,
    std::unique_ptr<localization::IPositionSolver> solver,
    std::vector<std::unique_ptr<localization::IPositionSender>> sender,
    std::optional<uint> port = std::nullopt, bool verbose = false);
void RunJointLocalization(
    MultiCameraDetector& detector_source,
    std::unique_ptr<localization::IJointPositionSolver> solver,
    std::unique_ptr<localization::IPositionSender> sender,
    bool verbose = false);
}  // namespace localization
