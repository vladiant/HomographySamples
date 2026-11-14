#include "rhdr_interface.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "rhdr_interface_priv.h"
extern "C" {
#include "rhdr.h"
#include "rhdr_prv.h"
}

#define SMEAR_CORE_SIZE (9)

class cTImg {
 public:
  unsigned short width;
  unsigned short height;
  unsigned short channels;
  unsigned char* buffer;
};

rhdr_context_t* rhdr_ctx = NULL;
rhdr_in_data_t rhdr_in;

uint16_t rhdr_smear_core[SMEAR_CORE_SIZE * SMEAR_CORE_SIZE] = {
    // SMEAR_SUM_MAX-4*SMEAR_SUM_STEP,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
};

rhdr_noise_vs_gain_t rhdr_noise_vs_gain[] = {
    //{slope, base}
    {36, (12288 >> 4)}, {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400},
};

static uint16_t const_gamma[256] = {
    0,   4,   8,   12,  15,  18,  21,  24,  26,  29,  32,  34,  36,  38,  41,
    43,  45,  47,  49,  51,  52,  54,  56,  58,  60,  61,  63,  65,  66,  68,
    69,  71,  72,  74,  75,  77,  78,  80,  81,  82,  84,  85,  87,  88,  89,
    90,  92,  93,  94,  96,  97,  98,  99,  100, 102, 103, 104, 105, 106, 107,
    109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 121, 122, 123, 124,
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 135, 136, 137, 138,
    139, 140, 141, 142, 143, 144, 145, 146, 147, 147, 148, 149, 150, 151, 152,
    153, 154, 154, 155, 156, 157, 158, 159, 160, 160, 161, 162, 163, 164, 164,
    165, 166, 167, 168, 169, 169, 170, 171, 172, 172, 173, 174, 175, 176, 176,
    177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 188,
    188, 189, 190, 191, 191, 192, 193, 193, 194, 195, 196, 196, 197, 198, 198,
    199, 200, 200, 201, 202, 202, 203, 204, 205, 205, 206, 207, 207, 208, 209,
    209, 210, 211, 211, 212, 213, 213, 214, 215, 215, 216, 216, 217, 218, 218,
    219, 220, 220, 221, 222, 222, 223, 224, 224, 225, 225, 226, 227, 227, 228,
    228, 229, 230, 230, 231, 232, 232, 233, 233, 234, 235, 235, 236, 236, 237,
    238, 238, 239, 239, 240, 241, 241, 242, 242, 243, 244, 244, 245, 245, 246,
    246, 247, 248, 248, 249, 249, 250, 250, 251, 252, 252, 253, 253, 254, 254,
    255};

rhdr_tune_data_t rhdr_tune_data = {
    {
// uint8 y_weight[256];
#include "pix_val_weight.txt"
    },
    // uint32 mov_dif_thr_const;
    0,
    // 5/256 = 2%                    //uint32 mov_dif_thr_mult;
    100,
    // SMEAR_CORE_SIZE,              //uint8  smear_core_size;
    0,
    // uint16*  smear_core;
    rhdr_smear_core,
    // uint8 rhdr_noise_vs_gain_size
    1,  // sizeof(rhdr_noise_vs_gain)/sizeof(rhdr_noise_vs_gain[0]),    //1,
        // //proposed by Stoyan
    // rhdr_noise_vs_gain_t* noise_vs_gain;
    rhdr_noise_vs_gain,
    2,  // uint8 dark_snr_thr;

    //    1,//uint32 percent_min;// 1/1024
    //    1,//uint32 percent_max;// 1/1024
    //    20,//uint8  max_sub;
    //    200,//uint8  max_target;
    //    512,//uint16 max_expand; //U16Q8
};

// Attempts to read the gain and exposure from the given info files
int hdr_read_exp_gain(unsigned long* img_exp_us, unsigned int* img_gain,
                      char** input, int input_count) {
  uint32_t temp_exp;
  uint16_t temp_gain;

  // Read the exposure and gain
  for (int i = 0; i < input_count; ++i) {
    if (read_exp_gain(&temp_exp, &temp_gain, input[i]) == 0) {
      img_exp_us[i] = temp_exp;
      img_gain[i] = temp_gain;
    } else {
      return (1);
    }
  }
  return (0);
}

// Fuses the images using the gain and exposure
int hdr_fuse(cTImg* img, char* outimg, unsigned long* img_exp_us,
             unsigned int* img_gain, int input_count, int ref_image) {
  // Set the HDR structure

  // enable
  rhdr_in.enable = 1;
  // image count
  rhdr_in.img_count = input_count;
  // raw bit depth
  rhdr_in.raw_bitsperpix = 16;  // needed for RGB HDR
  // pixels per line
  rhdr_in.img_ppln = img[0].width;
  rhdr_in.img_sx = img[0].width;
  rhdr_in.img_sy = img[0].height;
  // data pedestal in the RAW images
  rhdr_in.data_pedestal = 0;  // not used!!!
  rhdr_in.tune_data = (rhdr_tune_data_t*)&rhdr_tune_data;
  rhdr_in.ref_image = ref_image;

  memcpy(rhdr_in.in_gamma, const_gamma, sizeof(uint16_t) * 256);

  // Read the exposure and gain
  for (int i = 0; i < input_count; ++i) {
    // TO DO: check the range of input values
    rhdr_in.img_exps_us[i] = img_exp_us[i];
    rhdr_in.img_gains[i] = img_gain[i];
  }

  int err = 0;
  // Initialization of HDR routine
  err |= test_rhdr_create((void**)(&rhdr_ctx));

  // Convert images from 8bit to 16bit
  for (int i = 0; i < input_count; ++i) {
    rhdr_in.ptr_imgs[i] =
        (uint16_t*)malloc(rhdr_in.img_sx * rhdr_in.img_sy * 3 * 2);
    if (rhdr_in.ptr_imgs[i] == NULL) {
      printf("Can not allocate memory for input image %d bytes\n",
             rhdr_in.img_sx * rhdr_in.img_sy * 3 * 2);
      return (5);
    }
    rgb24_to_rgb48(img[i].buffer, rhdr_in.ptr_imgs[i], rhdr_in.img_sx,
                   rhdr_in.img_sy);
  }

  // Reserve memory for the output image
  rhdr_in.out_rgb = (uint16_t*)malloc(rhdr_in.img_sx * rhdr_in.img_sy * 3 * 2);
  if (rhdr_in.out_rgb == NULL) {
    printf("Can not allocate memory for output image %d bytes\n",
           rhdr_in.img_sx * rhdr_in.img_sy * 3 * 2);
    return (5);
  }

  // Perform HDR routine
  err |= test_rhdr_process(rhdr_ctx, &rhdr_in);

  // Release memory taken by input images
  for (int i = 0; i < rhdr_in.img_count; i++) {
    if (rhdr_in.ptr_imgs[i]) free(rhdr_in.ptr_imgs[i]);
  }

  // Apply gamma
  uint16_t gamma_hdr_temp[256] = {
#include "hdr_gamma_temp.txt"
  };
  uint32_t index;
  uint32_t slp;
  uint32_t r1;
  for (index = 0; index < rhdr_in.img_sx * rhdr_in.img_sy * 3; ++index) {
    r1 = rhdr_in.out_rgb[index] << (16 - RHDR_OUT_BITS_PER_PIX);
    if ((r1 >> 8) < 255) {
      slp = gamma_hdr_temp[(r1 >> 8) + 1] - gamma_hdr_temp[(r1 >> 8)];
    }
    r1 = ((uint32_t)gamma_hdr_temp[(r1 >> 8)] << 6) +
         (slp << 6) * (r1 & ((1 << 8) - 1)) / (1 << 8);
    *outimg++ = r1 / 256;

    // without gamma correction
    //*outimg++=rhdr_in.out_rgb[index]/256;
  }

  // Free initial output image
  if (rhdr_in.out_rgb) free(rhdr_in.out_rgb);

  // Free memory
  err |= test_rhdr_delete(rhdr_ctx);
  return err;
}
