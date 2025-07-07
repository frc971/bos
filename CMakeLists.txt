cmake_minimum_required(VERSION 3.10)
project(NerualNetwork CXX CUDA)

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

find_package(nlohmann_json REQUIRED)
find_package(wpilib REQUIRED)
find_package(OpenCV REQUIRED)

add_compile_options(-Wno-psabi)
add_subdirectory(971apriltag)
add_subdirectory(main)
