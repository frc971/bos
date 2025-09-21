#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/mat.hpp>
namespace Camera {

class Camera {
private:
  int id_;
public:
  void setId(int id) { id_ = id; }
  virtual void getFrame(cv::Mat& mat);
};
} // namespace Camera

#endif // CAMERA_H
