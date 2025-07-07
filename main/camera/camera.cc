#include "camera.h"
namespace Camera {

Camera::Camera(camera_info_t info)
    : info_(info), cap_(info.pipeline, cv::CAP_GSTREAMER) {}
Camera::~Camera() { cap_.release(); }

void Camera::getFrame(cv::Mat &mat) { cap_ >> mat; }

} // namespace Camera
