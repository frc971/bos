// #ifndef STREAMER_H
// #define STREAMER_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <arpa/inet.h>
namespace Camera {

class Streamer {
public:
  Streamer (uint port, bool verbose=false);
  void writeFrame(cv::Mat &mat);
  // ~Streamer();
  // void getFrame(cv::Mat &mat);

private:
  void Listen();

private:
  bool status_;
  int server_fd_;
  sockaddr_in address_;
  socklen_t address_length_;
  std::vector<int> client_fd_;
  std::thread listen_thread_;
};

} // namespace Camera

// #endif // STREAMER_H
