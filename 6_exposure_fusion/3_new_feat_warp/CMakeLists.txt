cmake_minimum_required (VERSION 3.22)

project(new_feat_warp)

find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)

add_library(
    test_algorithms_core
    src/test/ImageGridWarp/warpedGridIndex.c
    src/test/ImageGridWarp/warpedGridToUniform.c
    src/test/ImageGridWarp/ImageGridWarpAffine.c
    src/test/ImageGridWarp/ImageGridWarp.c
)

set_target_properties(
    test_algorithms_core
    PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(
    test_algorithms_core
    PUBLIC
        private_include
)

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
        test_algorithms_core
        ${OpenCV_LIBS}
        JPEG::JPEG
)

