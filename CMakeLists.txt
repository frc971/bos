cmake_minimum_required(VERSION 3.10)
project(BOS CXX C CUDA)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(CMAKE_CUDA20_STANDARD_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA20_EXTENSION_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA_FLAGS "--expt-relaxed-constexpr")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

include_directories(/usr/local/cuda-12.6/include)
link_directories(/usr/local/cuda-12.6/compat)

link_directories(/usr/lib/aarch64-linux-gnu/nvidia)

find_package(wpilib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "Wpilib dir: ${wpilib_DIR}")
message(STATUS "Eigen version found in root: ${Eigen3_VERSION}")

add_subdirectory(third_party/json)
add_subdirectory(third_party/971apriltag)
add_subdirectory(third_party/kalman-cpp)

add_compile_options(-Wno-psabi)
add_subdirectory(src)
