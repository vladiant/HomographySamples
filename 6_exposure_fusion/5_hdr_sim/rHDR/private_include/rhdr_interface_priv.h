

#pragma once

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

class cTImg;

// Reads a number from info file
int getnum(FILE* input_file);

// Basic routine to read gain and exposure data
int read_exp_gain(uint32_t* img_exp_us, uint16_t* img_gain, char* in_file_name);

// Conversion of 8bit to 16bit images
void rgb24_to_rgb48(unsigned char* src, uint16_t* dst, int width, int height);

#ifdef __cplusplus
}
#endif
