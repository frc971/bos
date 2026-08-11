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
