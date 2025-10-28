#include "rhdr_interface_priv.h"

#include <cstdint>
#include <cstring>

extern "C" {
#include "rhdr.h"
#include "rhdr_prv.h"
}

#define ERR_VALUE (-36842122)
#define EXPGAIN_FNAME_EXTENSION ".txt"

// Reads a number from info file
int getnum(FILE* input_file) {
  char line[256];
  int32_t result;
  int8_t success = 0;

  do {
    success = 0;
    if (NULL == fgets(line, 255, input_file)) {
      success = -1;
    }
    if (('-' == line[0]) || (('0' <= line[0]) && ('9' >= line[0]))) {
      success = 1;
    }
    if (0 == line[0]) {
      success = -1;
    }
  } while ((0 == success));

  if (-1 == success) {
    return ERR_VALUE;
  }

  sscanf(line, "%d", &result);

  return (result);
}

// Basic routine to read gain and exposure data
int read_exp_gain(uint32_t* img_exp_us, uint16_t* img_gain,
                  char* in_file_name) {
  char* point;
  char* slash;
  char exp_fname[280];
  FILE* f;
  int32_t dummy;

  strcpy(exp_fname, in_file_name);
  point = strrchr(exp_fname, '.');
  slash = strrchr(exp_fname, '\\');
  if (point > slash) {
    *point = 0;  // remove outimg_file_name extension
  }
  strcat(exp_fname, EXPGAIN_FNAME_EXTENSION);

  if (NULL == (f = fopen(exp_fname, "rt"))) {
    printf("Error opening exp/gain file %s !\n", exp_fname);
    return (1);
  }

  // format in OMAP4 simulator, gnerated by perl
  dummy = getnum(f);
  if (ERR_VALUE == dummy) {
    printf("Error reading exposure/gain file %s !\n", exp_fname);
    fclose(f);
    return (1);
  }
  *img_exp_us = dummy;

  dummy = getnum(f);
  if (ERR_VALUE == dummy) {
    printf("Error reading exposure/gain file %s !\n", exp_fname);
    fclose(f);
    return (1);
  }
  *img_gain = dummy;

  fclose(f);
  return (0);
}

// Conversion of 8bit to 16bit images
void rgb24_to_rgb48(unsigned char* src, uint16_t* dst, int width, int height) {
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src++;
      *dst++ = *src++;
      *dst++ = *src++;
    }
  }
}
