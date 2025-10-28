#pragma once

// OpenCV libraries
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

class cTImg;

// Prints help about program usage
void print_help(char* argv);

// Reads the command line arguments, configuration and image file names, checks
// them for consistency
void hdr_read(int argc, char** argv, char** input, char** output,
              int* input_count, int* ref_image);

// Reads the images, checks them for consistency and stores them in memory
void hdr_load_images(cv::Mat* initial_images, cTImg* img, char** input,
                     int input_count);

#ifdef __cplusplus
}
#endif
