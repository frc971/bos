cmake_minimum_required(VERSION 3.10)
project(NerualNetwork CXX C CUDA)

# set(CMAKE_BUILD_TYPE Debug)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(CMAKE_CUDA20_STANDARD_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA20_EXTENSION_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA_FLAGS "--expt-relaxed-constexpr")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")


add_subdirectory(third_party/nlohmann_json)
add_subdirectory(third_party/971apriltag)

find_package(wpilib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_compile_options(-Wno-psabi)
add_subdirectory(main)
