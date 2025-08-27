# genrule(
#     name = "cmake_build_allwpilib",
#     srcs = ["@opencv//file", "@opencv_contrib//file"],
#     outs = ["a.txt"],
#     cmd = """
# cp $(location @opencv//file) .
# cp $(location @opencv_contrib//file) .
# ls -lah
# unzip -qq opencv.zip
# unzip -qq opencv_contrib.zip
# ls -lah
# cd opencv-4.10.0
# mkdir release
# cd release/
# # cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="8.7" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.10.0/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
#     """,
# )
#
