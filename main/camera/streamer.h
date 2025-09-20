#ifndef BOS_MAIN_CAMERA_STREAMER_H_
#define BOS_MAIN_CAMERA_STREAMER_H_

#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
namespace Camera {

class Streamer {
 public:
  Streamer(uint port, bool verbose = false, uint skip_frame = 5);
  void WriteFrame(cv::Mat& mat);

 private:
  bool status_;
  bool verbose_;
  int skip_frame_;
  int skip_frame_idx_;
  int server_fd_;
  sockaddr_in address_;
  socklen_t address_length_;
  int client_fd_;
  std::thread listen_thread_;
};

}  // namespace Camera

#endif // STREAMER_H
