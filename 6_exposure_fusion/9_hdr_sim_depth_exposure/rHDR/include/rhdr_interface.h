#pragma once

#ifdef __cplusplus
extern "C" {
#endif

class cTImg;

// Attempts to read the gain and exposure from the given info files
int hdr_read_exp_gain(unsigned long* img_exp_us, unsigned int* img_gain,
                      char** input, int input_count);

// Fuses the images using the gain and exposure
int hdr_fuse(cTImg* img, char* outimg, unsigned long* img_exp_us,
             unsigned int* img_gain, int input_count, int ref_image);

#ifdef __cplusplus
}
#endif
