cmake_minimum_required(VERSION 3.20)

project(hdr_sim_depth_exposure)

find_package(OpenCV REQUIRED)

add_subdirectory(rHDR)

add_executable(
    ${PROJECT_NAME}
    main.cpp
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        HDRroutine
        ${OpenCV_LIBS}
)
