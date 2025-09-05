// https://gist.github.com/SteveRuben/2a15909e384b582c51b5
#include "streamer.h"
namespace Camera{


static std::string k_header = 
    "HTTP/1.0 200 OK\r\n"
    "Server: MJPEG-Streamer\r\n"
    "Cache-Control: no-cache\r\n"
    "Pragma: no-cache\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

Streamer::Streamer(uint port, bool verbose) : status_(false){
  address_length_ = sizeof(address_);

  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ == -1) { perror("socket"); return; }

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(port);

  if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
    perror("bind"); 
    return;
  }

  listen(server_fd_, 5);
  if (verbose){
    std::cout << "HTTP MJPEG server running on port " << port << std::endl;
  }
  listen_thread_  = std::thread(&Streamer::Listen, this);
  status_ = true;
}

void Streamer::Listen(){
  // TODO make thread safe
  while (true){
    int client_fd = accept(server_fd_, (struct sockaddr*)&address_, &address_length_);
    send(client_fd, k_header.c_str(), k_header.size(), 0);
    if (client_fd >= 0){
      client_fd_.push_back(client_fd);
    }
  }
}

void Streamer::writeFrame(cv::Mat &mat){
  cv::Mat compressed;
  cv::resize(mat, compressed, cv::Size(480, 480), 0, 0, cv::INTER_AREA);
  cv::cvtColor(compressed, compressed, cv::COLOR_BGR2GRAY);

  std::vector<uchar> buf;
  cv::imencode(".jpg", compressed, buf);

  std::string part = 
      "--frame\r\n"
      "Content-Type: image/jpeg\r\n"
      "Content-Length: " + std::to_string(buf.size()) + "\r\n\r\n";
  for (int client_fd : client_fd_){
    send(client_fd, part.c_str(), part.size(), 0);
    send(client_fd, reinterpret_cast<char*>(buf.data()), buf.size(), 0);
    send(client_fd, "\r\n", 2, 0);
  }
}
}

