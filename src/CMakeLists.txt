cmake_minimum_required(VERSION 3.10)

include_directories(${CMAKE_SOURCE_DIR})

if(ENABLE_CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy;
        -checks=modernize-*,performance-*;
        -warnings-as-errors=*;
    )
endif()

add_compile_options(-Wno-c99-designator)

add_subdirectory(utils)
add_subdirectory(camera)
#add_subdirectory(gamepiece)
add_subdirectory(localization)
add_subdirectory(calibration)
#add_subdirectory(yolo)
add_subdirectory(pathing)
add_subdirectory(test)

set(CMAKE_CXX_CLANG_TIDY "")
