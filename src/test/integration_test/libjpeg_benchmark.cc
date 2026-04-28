#include <chrono>
#include <iterator>
#include <numeric>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/flags/parse.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

ABSL_FLAG(std::string, image_path, "path/to/image.jpg",  // NOLINT
          "Path to the image file to be loaded and decoded.");

auto loadImage(const std::string& path) -> std::vector<uchar> {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    LOG(FATAL) << "Image could not be loaded from path '" << path
               << "'; path invalid or image doesn't exist.";
  }
  std::vector<uchar> data(std::istreambuf_iterator<char>(file), {});
  return data;
}

auto decodeImage(const std::vector<uchar>& raw) -> cv::Mat {
  cv::Mat decoded = cv::imdecode(raw, cv::IMREAD_COLOR);
  return decoded;
}

auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);
  std::string imagePath = absl::GetFlag(FLAGS_image_path);
  std::vector<uchar> rawImageData = loadImage(imagePath);
  // print which jpeg library we are using so that we know and don't get mixed up
  std::istringstream stream(cv::getBuildInformation());
  std::string line;
  while (std::getline(stream, line)) {
    if (line.find("jpeg:") != std::string::npos ||
        line.find("JPEG:") != std::string::npos) {
      LOG(INFO) << line;
    }
  }
  const uint tests = 1000;
  std::vector<double> trials;
  for (uint i = 0; i < tests; ++i) {
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat decodedImage = decodeImage(rawImageData);
    auto end = std::chrono::high_resolution_clock::now();
    if (decodedImage.empty()) {
      LOG(INFO) << "image decode failed";
      continue;
    }
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    trials.push_back(ms);
  }

  double total = std::accumulate(trials.begin(), trials.end(), 0.0);
  double average = total / trials.size();
  LOG(INFO) << "Average decoding time over " << tests << " trials: " << average
            << " ms";
  return 0;
}
