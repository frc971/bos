#pragma once

#include <arpa/inet.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>
namespace camera {

static const int MAX_CLIENTS = 5;
class Streamer {
 public:
  Streamer(uint port, bool verbose = false, uint skip_frame = 5);
  ~Streamer();
  void WriteFrame(cv::Mat& mat);

 private:
  bool status_;
  bool verbose_;
  int skip_frame_;
  int skip_frame_idx_;
  int server_fd_;
  sockaddr_in address_;
  socklen_t address_length_;
  int client_fds_[MAX_CLIENTS];
  std::thread* listen_thread_;  // Can we make this static and not ptr?
};

}  // namespace camera
