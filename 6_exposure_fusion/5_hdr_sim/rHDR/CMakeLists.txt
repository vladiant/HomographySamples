cmake_minimum_required(VERSION 3.20)

project(HDRroutine)

add_library(
    HDRroutine
    src/rhdr.c
    src/rhdr_interface.cpp
    src/rhdr_interface_priv.cpp
    aux/proc_img.c
)

set_target_properties(
    HDRroutine
    PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED YES
        C_EXTENSIONS NO
        LINKER_LANGUAGE C
)

target_include_directories(
    HDRroutine
    PUBLIC
        include
    PRIVATE
        aux
        inc
        osal
        data
        private_include
)

target_compile_options(
    HDRroutine
    PUBLIC
         -Wall -Wextra -Wpedantic
)
