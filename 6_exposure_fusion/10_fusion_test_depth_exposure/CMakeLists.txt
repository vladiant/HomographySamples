cmake_minimum_required(VERSION 3.20)

project(fusion_test_depth_exposure LANGUAGES C CXX)

find_package(OpenCV REQUIRED)

add_executable(
    ${PROJECT_NAME}
    rhdr_lib/src/rhdr.c
    rhdr_lib/src/proc_img.c
    main.cpp
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED YES
        C_EXTENSIONS NO
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        rhdr_lib/include
        rhdr_lib/src/private_include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
)
