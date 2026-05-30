#include <cuda_runtime_api.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "NvJpegDecoder.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

auto copy_plane(const NvBuffer::NvBufferPlane& plane, int rows, int cols,
                unsigned char* dst) -> bool {
  if (!plane.data) {
    return false;
  }

  for (int y = 0; y < rows; ++y) {
    memcpy(dst + (y * cols), plane.data + (y * plane.fmt.stride), cols);
  }

  return true;
}
auto main() -> int {
  const std::string input_path = "cat.jpg";
  const std::string output_path = "cat_nvjpeg.png";

  std::ifstream input_stream(input_path, std::ios::binary);
  if (!input_stream) {
    std::cerr << "Failed to open input JPEG: " << input_path << '\n';
    return 1;
  }

  std::vector<unsigned char> jpeg_bytes(
      (std::istreambuf_iterator<char>(input_stream)),
      std::istreambuf_iterator<char>());

  if (jpeg_bytes.empty()) {
    std::cerr << "Input file is empty: " << input_path << '\n';
    return 1;
  }

  NvJPEGDecoder* decoder = NvJPEGDecoder::createJPEGDecoder("nvjpeg_decoder");
  if (!decoder) {
    std::cerr << "Failed to create NvJPEGDecoder" << '\n';
    return 1;
  }

  NvBuffer* decoded_buffer = nullptr;
  uint32_t pixfmt = 0;
  uint32_t width = 0;
  uint32_t height = 0;

  const int decode_status =
      decoder->decodeToBuffer(&decoded_buffer, jpeg_bytes.data(),
                              jpeg_bytes.size(), &pixfmt, &width, &height);

  if (decode_status < 0 || !decoded_buffer) {
    std::cerr << "NvJPEGDecoder failed to decode: " << input_path << '\n';
    delete decoder;
    return 1;
  }

  if (pixfmt != V4L2_PIX_FMT_YUV420M || decoded_buffer->n_planes < 3) {
    std::cerr << "Unsupported decoded format. Expected YUV420M, got pixfmt="
              << pixfmt << " with planes=" << decoded_buffer->n_planes << '\n';
    delete decoded_buffer;
    delete decoder;
    return 1;
  }

  cv::Mat i420(height + (height / 2), width, CV_8UC1);

  unsigned char* y_dst = i420.ptr<unsigned char>(0);
  unsigned char* u_dst = i420.ptr<unsigned char>(height);
  unsigned char* v_dst = i420.ptr<unsigned char>(height + (height / 4));

  const bool y_ok =
      copy_plane(decoded_buffer->planes[0], static_cast<int>(height),
                 static_cast<int>(width), y_dst);
  const bool u_ok =
      copy_plane(decoded_buffer->planes[1], static_cast<int>(height / 2),
                 static_cast<int>(width / 2), u_dst);
  const bool v_ok =
      copy_plane(decoded_buffer->planes[2], static_cast<int>(height / 2),
                 static_cast<int>(width / 2), v_dst);

  if (!y_ok || !u_ok || !v_ok) {
    std::cerr << "Failed to copy decoded YUV planes" << '\n';
    delete decoded_buffer;
    delete decoder;
    return 1;
  }

  cv::Mat bgr;
  cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);

  if (!cv::imwrite(output_path, bgr)) {
    std::cerr << "Failed to write output image: " << output_path << '\n';
    delete decoded_buffer;
    delete decoder;
    return 1;
  }

  std::cout << "Decoded " << input_path << " with NvJPEGDecoder (" << width
            << "x" << height << ") and wrote " << output_path << '\n';

  delete decoded_buffer;
  delete decoder;
  return 0;
}
