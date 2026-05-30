find_library(NVJPEG_LIB
   NAMES nvjpeg
   PATHS
     /usr/lib/aarch64-linux-gnu/nvidia
)
add_library(nvjpeg
  NvJpegDecoder.cpp
  NvBuffer.cpp
  NvElement.cpp
  NvElementProfiler.cpp
  NvLogging.cpp
)

target_include_directories(nvjpeg PUBLIC 
  /usr/src/jetson_multimedia_api/include
  /usr/src/jetson_multimedia_api/include/libjpeg-8b
)

target_link_libraries(nvjpeg PUBLIC 
  ${NVJPEG_LIB}
  ${NVBUFSURFACE_LIB}
)
