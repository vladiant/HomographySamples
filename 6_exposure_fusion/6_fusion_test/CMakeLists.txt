cmake_minimum_required(VERSION 3.28)

project(fusion_test LANGUAGES C)

add_executable(
    ${PROJECT_NAME}
    rhdr_lib/src/rhdr.c
    rhdr_lib/src/proc_img.c
    main.c
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED YES
        C_EXTENSIONS NO
        LINKER_LANGUAGE C
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        rhdr_lib/include
        rhdr_lib/src/private_include
)

target_compile_options(
    ${PROJECT_NAME}
    PUBLIC
         -Wall -Wextra -Wpedantic
)

