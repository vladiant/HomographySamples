#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#define RHDR_OUT_BITS_PER_PIX (12)
#define RHDR_MAX_IMGS (5)

typedef struct {
  uint16_t slope;  // noise ampl / average pixle value, unity 256
  uint16_t base;   // noise in dark * 256 (assuming 8bit pixel values)
} rhdr_noise_vs_gain_t;

typedef struct {
  uint8_t y_weight[256];
  uint32_t mov_dif_thr_const;
  uint32_t mov_dif_thr_mult;
  uint8_t smear_core_size;
  uint16_t* smear_core;
  uint8_t noise_vs_gain_size;
  rhdr_noise_vs_gain_t* noise_vs_gain;
  uint8_t dark_snr_thr;
  //    uint32_t percent_min;// 1/1024
  //    uint32_t percent_max;// 1/1024
  //    uint8_t  max_sub;
  //    uint8_t  max_target;
  //    uint16_t max_expand;

} rhdr_tune_data_t;

typedef enum {
  RHDR_RAW_RGGB,
  RHDR_RAW_BGGR,
  RHDR_RAW_GRBG,
  RHDR_RAW_GBRG,
  RHDR_RAW_COLPTN_COUNT
} rhdr_raw_colptn_t;

typedef struct {
  uint8_t enable;
  uint16_t* ptr_imgs[RHDR_MAX_IMGS];    // RGB16bit images
  uint32_t img_exps_us[RHDR_MAX_IMGS];  // microseconds
  uint16_t img_gains[RHDR_MAX_IMGS];    // multiplier U16Q8
  uint16_t in_gamma[256];  // gamma of the input images, 256 8-bit values
  uint16_t* out_rgb;
  uint8_t img_count;
  uint8_t raw_bitsperpix;
  uint16_t img_ppln;
  uint16_t img_sx;
  uint16_t img_sy;
  uint16_t data_pedestal;
  rhdr_tune_data_t* tune_data;
  uint8_t ref_image;
} rhdr_in_data_t;

int32_t test_rhdr_create(void** context);
int32_t test_rhdr_process(void* context, rhdr_in_data_t* rhdr_in);
int32_t test_rhdr_delete(void* context);
