cmake_minimum_required (VERSION 3.22)

project (feat_warp)

find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)

add_executable(
    ${PROJECT_NAME} 
        main.cpp
        src/library.cpp
        src/imgconversion.c
        src/fuse.cpp
        src/cTJpeg.cpp
        src/common.cpp
        src/cTRegressions.cpp
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
        private_include
)

target_link_libraries(
    ${PROJECT_NAME}
        ${OpenCV_LIBS}
        JPEG::JPEG
)
