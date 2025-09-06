#include "streamer.h"
#include "camera.h"
int main(){
  Camera::Camera camera(Camera::CAMERAS.gstreamer1_30fps);
  cv::Mat frame;
  Camera::Streamer streamer(4972, true);
  while (true){
    camera.getFrame(frame);
    streamer.writeFrame(frame);
  }
}
