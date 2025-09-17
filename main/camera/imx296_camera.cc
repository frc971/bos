#include "imx296_camera.h"
namespace Camera {

IMX296Camera::IMX296Camera(camera_info_t info)
    : info_(info), cap_(info.pipeline, cv::CAP_GSTREAMER) {}
IMX296Camera::~IMX296Camera() {
  cap_.release();
}

void IMX296Camera::getFrame(cv::Mat& mat) {
  cap_ >> mat;
}

}  // namespace Camera
