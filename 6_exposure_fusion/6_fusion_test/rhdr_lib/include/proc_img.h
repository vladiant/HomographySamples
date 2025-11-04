#pragma once

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
  RAW_PTN_RGGB,
  RAW_PTN_GRBG,
  RAW_PTN_GBRG,
  RAW_PTN_BGGR,
  RAW_PTN_COUNT
} raw_colptn_t;

typedef struct {
  uint16_t k_awb_r;
  uint16_t k_awb_g;
  uint16_t k_awb_b;
  uint16_t* gamma;
  /* offsets befor CC - 10bit signed 2's complement */
  int16_t pre_offsets[3];  // 10bit

  /* ccm coeficients are S16Q8
  offsets assume 8bit pixel value
  R'   ccm[0], ccm[1], ccm[2]        R       offset[0]
  G' = ccm[3], ccm[4], ccm[5]   *    G   +   offset[1]
  B'   ccm[6], ccm[7], ccm[8]        B       offset[2]
  */
  int16_t ccm[9];  // S16Q8
  /* offsets after CC - 10bit signed 2's complement */
  int16_t post_offsets[3];  // 10bit
  /* YUV matrix conversion coeficients (after gamma)
  Y   yuv_m[0], yuv_m[1], yuv_m[2]        R
  U = yuv_m[3], yuv_m[4], yuv_m[5]   *    G
  V   yuv_m[6], yuv_m[7], yuv_m[8]        B
  */
  int16_t yuv_m[9];  // S16Q8
  raw_colptn_t col_ptn;
} proc_in_t;

void read_raw(uint16_t** img_raw, uint16_t sx, uint16_t sy, char* fname);

void read_bmp_16(uint16_t** img_raw, uint16_t sx, uint16_t sy, char* fname);

void read_bmp_16_noshift(uint16_t** img_raw, uint16_t sx, uint16_t sy,
                         char* fname);

void read_bmp(uint16_t** img_rgb, uint16_t* size_x, uint16_t* size_y,
              char* fname);
void process_raw(proc_in_t* proc_in, uint16_t sx, uint16_t sy, uint16_t ppln,
                 uint16_t* in_raw, uint16_t** out_rgb, uint8_t bits_per_pix);

int write_bmp(uint16_t* img_rgb, uint16_t sx, uint16_t sy, char* fname,
              char* fname_suff);

void write_raw(uint16_t* img_raw, uint16_t sx, uint16_t sy, char* fname);
