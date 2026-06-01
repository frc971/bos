#include "src/camera/nvidia_mjpeg_streamer.h"
#include <opencv2/imgproc.hpp>
#include "nvbufsurface.h"

namespace camera {
NvidiaMjpegStreamer::NvidiaMjpegStreamer(uint port)
    : streamer_(),
      encoder_(NvJPEGEncoder::createJPEGEncoder("streamer_encoder")) {
  streamer_.start(port);
}

void NvidiaMjpegStreamer::WriteFrame(const cv::Mat& mat) {
  size_t jpeg_size = mat.total() * mat.elemSize();
  // encoder_->setCropRect(0, 0, 0, 0);
  cv::Mat bgraMat;
  cv::cvtColor(mat, bgraMat, cv::COLOR_BGR2BGRA);

  NvBufSurfaceParamsEx a;
  NvBufSurfaceChromaSubsamplingParams b;
  NvBufSurfaceCreateParams c;
  int dmaFd;
  // NvBufferCreateParams params = {.width = bgraMat.cols,
  //                                .height = bgraMat.rows,
  //                                .payloadType = NvBufferPayload_SurfArray,
  //                                .memsize = (4 * bgraMat.cols * bgraMat.rows),
  //                                .colorFormat = NvBufferColorFormat_ARGB32,
  //                                .nvbuf_tag = NvBufferTag_NONE};

  Raw2NvBufSurface();

  // encoder_->encodeFromBuffer(&buffer, J_COLOR_SPACE color_space,
  //                            unsigned char** out_buf,
  //                            unsigned long& out_buf_size)
}
}  // namespace camera
