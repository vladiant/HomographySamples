#include <stdlib.h>
#include <string.h>

#include <iostream>

// OpenCV libraries
#include <opencv2/opencv.hpp>

// Image alignment libraries
#include <cTImg.h>
#include <library.h>

// HDR routine library
#include <rhdr_interface.h>

// HDR integration interface
#include <interface_hdr.h>

int main(int argc, char **argv) {
  // Align procedure main variables
  char *input[5];
  char *output[1];
  int input_count = 0;
  int ref_image = -1;

  // Program welcome message
  std::cout << "\nMulti Frame High Dynamic Range simulator\n";
  std::cout << std::endl;

  // Get file names
  hdr_read(argc, argv, input, output, &input_count, &ref_image);

  // Declare work images
  cv::Mat *initial_images = new cv::Mat[input_count];
  cTImg *img = new cTImg[input_count];

  // Load work images
  hdr_load_images(initial_images, img, input, input_count);

  cv::Mat out_image =
      cv::Mat(cv::Size(initial_images[0].cols, initial_images[0].rows),
              initial_images[0].depth(), initial_images[0].channels());

  unsigned long *image_exposure = new unsigned long[input_count];
  unsigned int *image_gain = new unsigned int[input_count];

  if (hdr_read_exp_gain(image_exposure, image_gain, input, input_count) == 0) {
    /*
     *  Align the images - last digit:
     *  0 - draw features and motion vectors
     *  1 - align images
     *  2 - fuse HDR
     */
    hdr_proc(img, input_count, &ref_image, 1);

    // Show images aligned
    // for(int i = 0; i < input_count; ++i)
    // {
    //         cv::namedWindow(input[i], 0);
    //         cv::imshow(input[i], initial_images[i]);
    //         cv::waitKey(0);
    //         cv::destroyWindow(input[i]);
    // }

    // Show gain and exposure data
    // for(int i = 0; i < input_count; ++i)
    // {
    //         std::cout << image_exposure[i] << "  ";
    //         std::cout << image_gain[i] << std::endl;
    // }

    out_image =
        cv::Mat(cv::Size(initial_images[0].cols, initial_images[0].rows),
                CV_8UC3, cv::Scalar(0, 0, 0));

    // Fuse the images via HDR algorithm
    hdr_fuse(img, reinterpret_cast<char *>(out_image.data), image_exposure,
             image_gain, input_count, ref_image);

  } else {
    std::cout << "Exposure and gain information unavailable." << std::endl;
    std::cout << "HDR fuse algorithm from alignment routine used." << std::endl;
    hdr_proc(img, input_count, &ref_image, 2);
    std::cout << "ref_image: " << ref_image << std::endl;
    initial_images[0].copyTo(out_image);
  }

  // Show output image
  cv::cvtColor(out_image, out_image, cv::COLOR_BGR2RGB);  // Nota Bene !!!
  cv::namedWindow(output[0], 0);
  cv::imshow(output[0], out_image);
  cv::waitKey(0);
  cv::destroyWindow(output[0]);

  // Save output image
  cv::imwrite(output[0], out_image);
  std::cout << "Output image saved." << std::endl;

  // Release memory
  delete[] initial_images;
  delete[] img;
  delete[] image_exposure;
  delete[] image_gain;

  return (0);
}
