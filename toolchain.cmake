set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the cross-compilers
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_AR aarch64-linux-gnu-ar)
set(CMAKE_RANLIB aarch64-linux-gnu-ranlib)

set(CMAKE_CROSSCOMPILING TRUE)

set(TARGET_FS /l4t/targetfs)
set(CMAKE_SYSROOT "${TARGET_FS}")
set(CMAKE_FIND_ROOT_PATH "${TARGET_FS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_FLAGS "--sysroot=${CMAKE_SYSROOT} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "--sysroot=${CMAKE_SYSROOT} ${CMAKE_CXX_FLAGS}")

include_directories(SYSTEM ${TARGET_FS}/usr/local/include)
include_directories(SYSTEM /l4t/targetfs/usr/local/cuda-12.6/targets/aarch64-linux/include)

set(CMAKE_CUDA_HOST_COMPILER aarch64-linux-gnu-g++)
