#include "realsense_camera.h"

namespace Camera {
RealSenseCamera::RealSenseCamera(int id) 
    : pipe_(), 
      frames_(pipe_.wait_for_frames()), 
      color_image_(frames_.get_color_frame()),
      depth_image_(frames_.get_depth_frame()) 
{
  setId(id);
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  frames_ = pipe_.wait_for_frames();
  color_image_ = frames_.get_color_frame();
  if (rs2::video_frame vid_frame = color_image_.as<rs2::video_frame>()) {
    mat = cv::Mat(cv::Size(vid_frame.get_width(), vid_frame.get_height()), CV_8UC3, (void*)color_image_.get_data(), cv::Mat::AUTO_STEP).clone();
  }
}
} // namespace Camera
