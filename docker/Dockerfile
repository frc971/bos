FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS builder

WORKDIR /
RUN apt-get update
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3.10-dev python3-numpy build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2 clangd ninja-build clang-format ripgrep gh libblas-dev liblapack-dev rsync vim stow curl unzip zip clang-tidy clang

RUN mkdir opencv

WORKDIR /opencv
RUN apt-get -y purge *libopencv*
RUN curl -L https://github.com/opencv/opencv/archive/4.10.0.zip -o opencv-4.10.0.zip
RUN curl -L https://github.com/opencv/opencv_contrib/archive/4.10.0.zip -o opencv_contrib-4.10.0.zip
RUN unzip opencv-4.10.0.zip
RUN unzip opencv_contrib-4.10.0.zip
RUN rm *.zip
RUN mkdir opencv-4.10.0/release
WORKDIR /opencv/opencv-4.10.0/release
RUN cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="8.7" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.10.0/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
RUN make -j$(nproc)
RUN make install


WORKDIR /allwpilib
RUN git clone https://github.com/wpilibsuite/allwpilib.git
WORKDIR /allwpilib/allwpilib
RUN apt-get install -y ninja-build
RUN apt-get install -y protobuf-compiler
RUN apt-get install -y libxrandr-dev
RUN apt-get install -y libssh-dev
RUN cmake --preset default -DWITH_GUI=OFF -DWITH_JAVA=OFF -DWITH_SIMULATION_MODULES=OFF -DWITH_TESTS=OFF
WORKDIR /allwpilib/allwpilib/build-cmake 
RUN cmake --build . --parallel 4 
RUN cmake --build . --target install
COPY wpilib-config.cmake /usr/local/share/wpilib/wpilib-config.cmake

WORKDIR /perf
RUN wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.0/sources/public_sources.tbz2 
RUN gunzip -c /proc/config.gz | grep PERF
RUN tar -xjf public_sources.tbz2
RUN apt-get install -y libperl-dev
RUN apt-get install -y flex
RUN apt-get install -y bison
RUN apt-get install -y libslang2-dev
RUN apt-get install -y libbfd-dev
RUN apt-get install -y tree
RUN tar -xjf Linux_for_Tegra/source/kernel_src.tbz2
WORKDIR kernel/kernel-jammy-src/tools/perf
RUN make -j$(nproc)
RUN cp perf /usr/bin

WORKDIR /eigen
RUN git clone https://gitlab.com/libeigen/eigen.git
WORKDIR eigen
RUN mkdir build
WORKDIR build
RUN cmake ..
RUN make -j$(nproc)
RUN make install

WORKDIR /releasesense
RUN apt-get install -y lsb-release
RUN apt-get install -y software-properties-common
RUN apt-get install -y libfastrtps-dev
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-utils librealsense2-dev
COPY realsense2Targets.cmake /usr/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Targets.cmake

WORKDIR /usr/local/bin
RUN wget https://github.com/neovim/neovim/releases/download/v0.11.4/nvim-linux-arm64.tar.gz
RUN tar -xzvf nvim-linux-arm64.tar.gz
RUN ln -s nvim-linux-arm64/bin/nvim nvim

WORKDIR /
COPY lib/* /usr/lib/

WORKDIR /
RUN curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
RUN curl -s --compressed -o /etc/apt/sources.list.d/ctr2025.list "https://deb.ctr-electronics.com/ctr2025.list"
RUN apt-get update && apt-get install -y phoenix6

WORKDIR /
