add_library(CscoreStreamer)
target_sources(CscoreStreamer
  PRIVATE
    cscore_streamer.cc
)
target_link_libraries(CscoreStreamer PRIVATE
  utils
)

add_library(DiskCamera)
target_sources(DiskCamera
  PRIVATE
    disk_camera.cc
)
target_link_libraries(DiskCamera PRIVATE
  utils
)

add_library(CvCamera)
target_sources(CvCamera
  PRIVATE
    cv_camera.cc
    write_frame.cc
    write_frame.cc
)
target_link_libraries(CvCamera PRIVATE
  utils
)

add_library(SelectCamera)
target_sources(SelectCamera
  PRIVATE
    select_camera.cc
)
target_link_libraries(SelectCamera PRIVATE
  DiskCamera
  CvCamera
  utils
)

add_library(CameraSource)
target_sources(CameraSource
  PRIVATE
    camera_source.cc
)
target_link_libraries(CameraSource PRIVATE
  utils
)

add_library(CameraConstants)
target_sources(CameraConstants
  PRIVATE
    camera_constants.cc
)
target_link_libraries(CameraConstants PRIVATE
  utils
)

add_library(MultiCameraSource)
target_sources(MultiCameraSource
  PRIVATE
    multi_camera_source.cc
)
target_link_libraries(MultiCameraSource PRIVATE
  utils
)


add_library(UVCCamera)
target_sources(UVCCamera
  PRIVATE
    uvc_camera.cc
)
target_link_libraries(UVCCamera PRIVATE
  uvc
  absl::status
  utils
  /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so
)
set(JETSON_API /usr/src/jetson_multimedia_api)
target_sources(UVCCamera PRIVATE
    ${JETSON_API}/samples/common/classes/NvJpegDecoder.cpp
    ${JETSON_API}/samples/common/classes/NvBuffer.cpp
    ${JETSON_API}/samples/common/classes/NvElement.cpp
    ${JETSON_API}/samples/common/classes/NvElementProfiler.cpp
    ${JETSON_API}/samples/common/classes/NvLogging.cpp
)
target_include_directories(UVCCamera PRIVATE
    ${JETSON_API}/include
    ${JETSON_API}/include/libjpeg-8b
    ${JETSON_API}/samples/common/classes
)

add_library(camera INTERFACE)
target_link_libraries(camera INTERFACE
  CscoreStreamer
  DiskCamera
  CvCamera
  SelectCamera
  CameraSource
  CameraConstants
  MultiCameraSource
  UVCCamera
)
