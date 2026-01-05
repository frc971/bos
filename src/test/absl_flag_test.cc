#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

ABSL_FLAG(int, port, 8080, "Port to listen on");  // NOLINT

// Demenstration of absl flags
auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);
  int port = absl::GetFlag(FLAGS_port);
  std::cout << port;
}
