add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc select_camera.cc camera_source.cc write_frame.cc camera_constants.cc multi_camera_source.cc)
set(JETSON_API /usr/src/jetson_multimedia_api)
target_sources(camera PRIVATE
    uvc_camera.cc
    ${JETSON_API}/samples/common/classes/NvJpegDecoder.cpp
    ${JETSON_API}/samples/common/classes/NvBuffer.cpp
    ${JETSON_API}/samples/common/classes/NvElement.cpp
    ${JETSON_API}/samples/common/classes/NvElementProfiler.cpp
    ${JETSON_API}/samples/common/classes/NvLogging.cpp
)

target_include_directories(camera PRIVATE
    ${JETSON_API}/include
    ${JETSON_API}/include/libjpeg-8b
    ${JETSON_API}/samples/common/classes
)
target_link_libraries(camera PRIVATE
    utils
    absl::status
    uvc
    /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so
)
