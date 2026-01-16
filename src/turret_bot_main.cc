#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  camera::CameraSource front_right_camera = camera::CameraSource(
      "front_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
              .pipeline)));

  camera::CameraSource front_left_camera = camera::CameraSource(
      "front_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]
              .pipeline)));

  camera::CameraSource back_right_camera = camera::CameraSource(
      "back_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::IMX296_0].pipeline)));

  camera::CameraSource back_left_camera = camera::CameraSource(
      "back_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::IMX296_1].pipeline)));

  std::thread front_right_thread(
      localization::run_localization, std::ref(front_right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_right_camera.GetFrame().cols,
          front_right_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_FRONT_RIGHT),
      camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
          .extrinsics_path,
      4971, false);

  std::thread front_left_thread(
      localization::run_localization, std::ref(front_left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_left_camera.GetFrame().cols, front_left_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_FRONT_LEFT),
      camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]
          .extrinsics_path,
      4972, false);

  std::thread back_right_thread(
      localization::run_localization, std::ref(back_right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          back_right_camera.GetFrame().cols, back_right_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_BACK_RIGHT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_BACK_RIGHT),
      camera::camera_constants[camera::Camera::TURRET_BOT_BACK_RIGHT]
          .extrinsics_path,
      4973, false);

  std::thread back_left_thread(
      localization::run_localization, std::ref(back_left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          back_left_camera.GetFrame().cols, back_left_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_BACK_LEFT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_BACK_LEFT),
      camera::camera_constants[camera::Camera::TURRET_BOT_BACK_LEFT]
          .extrinsics_path,
      4974, false);

  // camera::CscoreStreamer front_right_streamer("front_right", 4971, 30, 1080,
  //                                             1080);
  //
  // camera::CscoreStreamer front_left_streamer("front_left", 4972, 30, 1080,
  //                                            1080);
  //
  // camera::CscoreStreamer back_right_streamer("back_right_streamer", 4973, 30,
  //                                            1080, 1080);
  //
  // camera::CscoreStreamer back_left_streamer("back_left_streamer", 4974, 30,
  //                                           1080, 1080);
  // while (true) {
  //   {
  //     cv::Mat frame = front_right_camera.GetFrame();
  //     front_right_streamer.WriteFrame(frame);
  //   }
  //   {
  //     cv::Mat frame = front_left_camera.GetFrame();
  //     front_left_streamer.WriteFrame(frame);
  //   }
  //   {
  //     cv::Mat frame = back_right_camera.GetFrame();
  //     back_right_streamer.WriteFrame(frame);
  //   }
  //   {
  //     cv::Mat frame = back_left_camera.GetFrame();
  //     back_left_streamer.WriteFrame(frame);
  //   }
  // }

  front_right_thread.join();
}
