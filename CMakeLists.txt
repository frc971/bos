cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

if(POLICY CMP0146)
  # Keep compatibility with packages that still use FindCUDA.
  cmake_policy(SET CMP0146 OLD)
endif()

set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.6/bin/nvcc")

project(BOS CXX C CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(CMAKE_CUDA20_STANDARD_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA20_EXTENSION_COMPILE_OPTION "-std=c++20")
set(CMAKE_CUDA_FLAGS "--expt-relaxed-constexpr")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

add_compile_options(-march=armv8-a+simd)

find_package(wpilib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vpi REQUIRED)
find_package(libuvc REQUIRED)

add_subdirectory(third_party/json)
add_subdirectory(third_party/971apriltag)
add_subdirectory(third_party/abseil-cpp)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(googletest)

add_compile_options(-Wno-psabi)
add_subdirectory(src)
