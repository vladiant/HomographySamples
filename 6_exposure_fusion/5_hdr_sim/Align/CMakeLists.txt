cmake_minimum_required (VERSION VERSION 3.20)

project (kltMatchFuse)

find_package(OpenCV REQUIRED)

add_library(
    kltMatchFuse
        src/library.cpp
        src/imgconversion.c
        src/fuse.cpp src/cTJpeg.cpp
        src/common.cpp
        src/cTRegressions.cpp
)

set_target_properties(
    kltMatchFuse
    PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(
    kltMatchFuse
    PUBLIC
        include
        private_include
)

target_link_libraries(
    kltMatchFuse
        ${OpenCV_LIBS}
)
