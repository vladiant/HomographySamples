 
cmake_minimum_required (VERSION VERSION 3.20)

project(HDRsimX)

find_package(OpenCV REQUIRED)

add_subdirectory(Align)
add_subdirectory(rHDR)

add_executable(
    HDRsimX
       src/interface_hdr.cpp
       main.cpp)

target_include_directories(
    HDRsimX
    PUBLIC
       include
)

target_link_libraries(
    HDRsimX 
       kltMatchFuse 
       HDRroutine
       ${OpenCV_LIBS}
)

target_compile_options(
    HDRsimX
    PUBLIC
         -Wall -Wextra -Wpedantic
) 
