#pragma once

#ifdef __ANDROID
#include <android/log.h>
#include <jni.h>
#else

#endif

#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>

// #define VARIABLE_LENGTH_ARRAYS
#define MAX_IMAGES 5

#define LOG_TAG "hdrLib"
#ifdef __ANDROID
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...) fprintf(stdout, __VA_ARGS__)
#define LOGE(...) fprintf(stderr, __VA_ARGS__)
#endif

#define getTime() hdr_getTime()
#define getuTime() hdr_getuTime()
long hdr_getTime(void);
long hdr_getuTime(void);
