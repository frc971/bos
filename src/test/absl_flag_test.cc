#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

ABSL_FLAG(int, port, 8080, "Port to listen on");

// Demenstration of absl flags
int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  int port = absl::GetFlag(FLAGS_port);
  std::cout << port;
}
