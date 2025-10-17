#include "pch.h"

// https://gist.github.com/SteveRuben/2a15909e384b582c51b5
#include "streamer.h"
#include <opencv2/imgcodecs.hpp>
namespace camera {

static std::string k_header =
    "HTTP/1.0 200 OK\\r\\n"
    "Server: MJPEG-Streamer\\r\\n"
    "Cache-Control: no-cache\\r\\n"
    "Pragma: no-cache\\r\\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\\r\\n\\r\\n";

Streamer::Streamer(uint port, bool verbose, uint skip_frame)
    : status_(false),
      verbose_(verbose),
      skip_frame_(skip_frame),
      skip_frame_idx_(0) {
  address_length_ = sizeof(address_);

  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ == -1) {
    std::cout << "failed to open socket!\\n";
    return;
  }

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(port);

  if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
    std::cout << "failed to bind!";
    return;
  }

  listen(server_fd_, 5);
  if (verbose) {
    std::cout << "HTTP MJPEG server running on port " << port << std::endl;
  }
  std::cout << "waiting for client..." << std::endl;
  for (int i = 0; i < MAX_CLIENTS; i++){
    client_fds_[i] = -1;
  }
  listen_thread_ = new std::thread([this](){
    while (true){
      int client_fd = accept(server_fd_, (struct sockaddr*)&address_, &address_length_);
      if (client_fd < 0) continue;
      for (int i = 0; i < MAX_CLIENTS; i++){
        if (client_fds_[i] == -1){
          client_fds_[i] = client_fd;
          send(client_fd, k_header.c_str(), k_header.size(), 0);
          if (verbose_){
            std::cout << "Got new connection with client_fd: " << client_fd << "\\n";
          }
        }
      }
    }
  });
  status_ = true;
}

void Streamer::WriteFrame(cv::Mat& frame) {
  std::vector<uchar> buf; // TODO make this part of the private variables so we do not need to malloc every time?
  cv::imencode(".jpg", frame, buf);

  std::string part =
      "--frame\\r\\n"
      "Content-Type: image/jpeg\\r\\n"
      "Content-Length: " +
      std::to_string(buf.size()) + "\\r\\n\\r\\n";
  for (int i = 0; i < MAX_CLIENTS; i++){
    if (client_fds_[i] != -1){
      send(client_fds_[i], part.c_str(), part.size(), 0);
      send(client_fds_[i], reinterpret_cast<char*>(buf.data()), buf.size(), 0);
      if (send(client_fds_[i], "\\r\\n", 2, 0) == -1){
        close(client_fds_[i]);
        client_fds_[i] = -1;
      }
    }
  }
}

Streamer::~Streamer(){
  close(server_fd_);
}
}  // namespace camera