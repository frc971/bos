#include "src/camera/select_camera.h"

namespace camera {
CVCamera SelectCamera() {
  return CVCamera(cv::VideoCapture(""));  // TODO
}

}  // namespace camera
