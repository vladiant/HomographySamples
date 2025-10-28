#include "interface_hdr.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

// OpenCV libraries
#include <opencv2/opencv.hpp>

// Image alignment libraries
#include <cTImg.h>

#ifndef TRUE
#define TRUE 1 == 1
#define FALSE 0 == 1
#endif

// RHDR_MAX_IMGS from HDR routine
#define MAX_IN_OUT_FILES 5

const char *DEFAULT_OUTPUT_FILE = "HDRresult.tiff";

// Reads the command line arguments, configuration and image file names, checks
// them for consistency
void hdr_read(int argc, char **argv, char **input, char **output,
              int *input_count, int *ref_image) {
  output[0] = (char *)DEFAULT_OUTPUT_FILE;

  int c;
  bool help_flag = TRUE;
  while ((c = getopt(argc, argv, "i:o:r:")) != -1) {
    help_flag = FALSE;
    switch (c) {
      case 'i':

        if ((*input_count) < MAX_IN_OUT_FILES) {
          if (optarg != 0) input[(*input_count)++] = strdup(optarg);
        } else {
          std::cout << "Input up to " << MAX_IN_OUT_FILES << " images.\n";
          std::cout << "Only the first " << MAX_IN_OUT_FILES
                    << " images will be used.\n";
          std::cout << "File " << strdup(optarg) << " skipped!\n" << std::endl;
          break;
        }
        break;
      case 'r': {
        if ((*input_count) < MAX_IN_OUT_FILES) {
          if (optarg != 0) input[(*input_count)++] = strdup(optarg);
        } else {
          std::cout << "Input up to " << MAX_IN_OUT_FILES << " images.\n";
          std::cout << "Only the first " << MAX_IN_OUT_FILES
                    << " images will be used.\n";
          std::cout << "File " << strdup(optarg) << " skipped!\n" << std::endl;
          break;
        }
        (*ref_image) = (*input_count) - 1;
      } break;
      case 'o':
        if (optarg != 0) output[0] = strdup(optarg);
        break;
      default:
        break;
    }
  }

  if (help_flag) {
    print_help(argv[0]);
    exit(0);
  }

  if (!strcmp("", output[0])) output[0] = (char *)DEFAULT_OUTPUT_FILE;

  // Show entered filenames
  std::cout << "Input Files:" << std::endl;
  for (int i = 0; i < (*input_count); ++i)
    std::cout << '\t' << i << ": " << input[i] << std::endl;
  std::cout << "Output File:" << std::endl;
  std::cout << '\t' << output[0] << std::endl;

  // Check parameteres for consistency
  if ((*input_count) < 2) {
    std::cerr << "\nAt least two files are required for HDR!\n" << std::endl;
    exit(1);
  }

  for (int i = 0; i < (*input_count); ++i) {
    for (int j = i + 1; j < (*input_count); ++j) {
      if (strcmp(input[i], input[j]) == 0) {
        std::cerr << "\nDuplicate file name: " << input[i] << '\n' << std::endl;
        exit(2);
      }
    }
  }
}

// Prints help about program usage
void print_help(char *argv) {
  std::cout << "Usage: " << argv
            << " -c config_file -i input_file -i input_file -r input_file ... "
               "<-t tuning file> <-o tuning file> \n\n";
  std::cout << std::endl;
  std::cout << "-c config_file: if missing, 3264x2448 resolution, 10 bits per "
               "pix, RED fist pixel and 0 data_pedestal are assumed\n";
  std::cout << "Used while reading RAW files. Currently disabled!\n";
  std::cout << std::endl;
  std::cout << "-i input_file: this statement can be repeated maximum "
            << MAX_IN_OUT_FILES << " times for inputing up to "
            << MAX_IN_OUT_FILES << " images\n";
  std::cout << std::endl;
  std::cout << "-r reference_file: this input file is used as a reference; if "
               "missing an analysis is performed to select it\n";
  std::cout << std::endl;
  std::cout
      << "-t tuning_file: if missing the values in rhdr_tune_data are used\n";
  std::cout << "At present it is not implemented\n";
  std::cout << std::endl;
  std::cout << "-o output_file: if missing it is set to HDRresult.jpg\n";
  std::cout << std::endl;
  std::cout << "This simulator also performs simple processing of RAW files to "
               "produce BMP.\n";
  std::cout << "AWB out file is needed for this - it should be named on one of "
               "the input images like img_1.awbout.\n";
  std::cout << "Currently disabled!\n";
  std::cout << std::endl;
}

// Reads the images, checks them for consistency and stores them in memory
void hdr_load_images(cv::Mat *initial_images, cTImg *img, char **input,
                     int input_count) {
  // Size of the images
  int size_x, size_y;

  // Load work images
  for (int i = 0; i < input_count; ++i) {
    if ((initial_images[i] = cv::imread(
             (const char *)input[i], cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR))
            .empty()) {
      std::cerr << "\nCan't open " << input[i] << " !" << std::endl;
      exit(3);
    }

    if (initial_images[i].channels() != 1) {
      // Remove alpha channel if any
      if (initial_images[i].channels() == 4) {
        cv::Mat temp_image = initial_images[i].clone();

        initial_images[i] = cv::Mat(cv::Size(temp_image.cols, temp_image.rows),
                                    temp_image.depth(), 3);
        cv::cvtColor(temp_image, initial_images[i], cv::COLOR_BGRA2BGR);
      }

      if (initial_images[i].channels() == 2) {
        std::cerr << "\nTwo channel images are not supported!" << std::endl;
        exit(0);
      }

      // OpenCV loads as BGR, however RGB is needed
      cv::cvtColor(initial_images[i], initial_images[i], cv::COLOR_BGR2RGB);
    } else {
      // RGB images needed
      std::cout << "Converting " << input[i] << " to RGB format" << std::endl;
      // cv::cvtColor(initial_images[i], initial_images[i], cv::COLOR_GRAY2RGB);
      cv::Mat temp_image = initial_images[i].clone();

      initial_images[i] = cv::Mat(cv::Size(temp_image.cols, temp_image.rows),
                                  temp_image.depth(), 3);
      cv::cvtColor(temp_image, initial_images[i], cv::COLOR_GRAY2RGB);
    }

    img[i].width = initial_images[i].cols;
    img[i].height = initial_images[i].rows;
    img[i].channels = initial_images[i].channels();
    img[i].buffer = (unsigned char *)initial_images[i].data;

    /*
    // Show loaded images
    cv::namedWindow(input[i], 0);
    cv::imshow(input[i], initial_images[i]);
    cv::waitKey(0);
    cv::destroyWindow(input[i]);
    */
  }

  // Check for consistency
  for (int i = 0; i < input_count; ++i) {
    if (i == 0) {
      size_x = initial_images[0].cols;
      size_y = initial_images[0].rows;
    } else if ((size_x != initial_images[i].cols) ||
               (size_y != initial_images[i].rows)) {
      std::cerr << "\nImages must be with equal size!" << std::endl;
      std::cerr << "Size of " << input[0] << " : " << size_x << "  " << size_y
                << std::endl;
      std::cerr << "Size of " << input[i] << " : " << initial_images[i].cols
                << "  " << initial_images[i].rows << '\n'
                << std::endl;
      exit(4);
    }
  }
}
