#include "imx296_camera.h"
#include <opencv2/core.hpp>
namespace camera {

IMX296Camera::IMX296Camera(camera_info_t info)
    : info_(info), cap_(info.pipeline, cv::CAP_GSTREAMER) {}
IMX296Camera::~IMX296Camera() {
  cap_.release();
}

void IMX296Camera::GetFrame(cv::Mat& mat) {
  cap_ >> mat;
  cv::flip(mat, mat, -1);
}

}  // namespace camera
