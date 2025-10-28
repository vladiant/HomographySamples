#include "proc_img.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IM_S_X 36
#define IM_S_Y 128
#define FILE_SIZE (IM_S_X * IM_S_Y * 3 + 54 + 2)
uint8_t head[54] = {
    // core header
    0x42, 0x4D,  //"BM"
    (FILE_SIZE & 0xFF), ((FILE_SIZE >> 8) & 0xFF),
    ((FILE_SIZE >> 16) & 0xFF),  // file size [byte] = image*3 + header + 2
    0x00, 0x00, 0x00,            // reserved_1
    0x00, 0x00,                  // reserved_2
    0x36, 0x00, 0x00, 0x00,      //?

    // info header
    0x28, 0x00, 0x00, 0x00,  // info header size [bute]
    (uint8_t)(IM_S_X & 0xFF), (uint8_t)(IM_S_X >> 8), 0x00,
    0x00,  // image size X [pixels]
    (uint8_t)(IM_S_Y & 0xFF), (uint8_t)(IM_S_Y >> 8), 0x00,
    0x00,                    // image size Y [pixels]
    0x01, 0x00,              // plain (set to 0x0001)
    0x18, 0x00,              // bits/pixel
    0x00, 0x00, 0x00, 0x00,  // compression metod (don't use)
    0x00, 0x00, 0x00, 0x00,  // number of bytes in the image data
    0xC3, 0x0E, 0x00, 0x00,  // resolution X [pixels/meter]
    0xC3, 0x0E, 0x00, 0x00,  // resolution Y [pixels/meter]
    0x00, 0x00, 0x00, 0x00,  // number of used colours
    0x00, 0x00, 0x00, 0x00   // number of important colours
};

/* ============================================================================
* void read_raw()
============================================================================ */
void read_raw(uint16_t** img_raw, uint16_t sx, uint16_t sy, char* fname) {
  FILE* f;
  uint16_t* img;
  uint32_t pixels_read;

  f = fopen(fname, "rb");
  if (f == NULL) {
    printf("Error opening raw image %s\n", fname);
    exit(1);
  }

  img = (uint16_t*)malloc(sx * sy * 2);
  if (img == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * sy * 2);
    exit(1);
  }

  pixels_read = fread(img, 2, sx * sy, f);

  if (pixels_read != sx * sy) {
    printf("Error reading raw image %s\n", fname);
    exit(1);
  }

  *img_raw = img;
  fclose(f);
}

/* ============================================================================
* void read_bmp_16()
============================================================================ */
void read_bmp_16(uint16_t** img_raw, uint16_t sx, uint16_t sy, char* fname) {
  FILE* f;
  int row, col;
  uint16_t* img;
  uint8_t* pin;
  uint16_t* pout;
  uint8_t* row_buf;
  uint8_t bmp_head[54];
  uint16_t sx_1, sy_1;

  f = fopen(fname, "rb");
  if (f == NULL) {
    printf("Error opening raw image %s\n", fname);
    exit(1);
  }

  if (fread(&bmp_head, 1, sizeof(bmp_head), f) != sizeof(bmp_head)) {
    printf("Error reading input image %s\n", fname);
    fclose(f);
    exit(1);
  }
  sx_1 = bmp_head[18] + ((uint16_t)bmp_head[19] << 8);
  sy_1 = bmp_head[22] + ((uint16_t)bmp_head[23] << 8);

  if (sx_1 != sx || sy_1 != sy) {
    printf(
        "Error: BMP size does not match specified size!\nBMP: %d %d\nExpected: "
        "%d %d",
        sx_1, sy_1, sx, sy);
    exit(1);
  }

  img = (uint16_t*)malloc(sx * sy * 3 * 2);
  if (img == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * sy * 3);
    exit(1);
  }

  row_buf = (uint8_t*)malloc(sx * 3);

  for (row = sy - 1; row >= 0; row--) {
    // read 1 rows
    fread(row_buf, 1, sx * 3, f);

    pin = row_buf;

    pout = img + 3 * row * sx;
    for (col = 0; col < sx; col++) {
      register uint16_t r, g, b;
      b = (uint16_t)(*pin++) << 8;
      g = (uint16_t)(*pin++) << 8;
      r = (uint16_t)(*pin++) << 8;
      *pout++ = r;
      *pout++ = g;
      *pout++ = b;
    }
  }

  *img_raw = img;
  fclose(f);
}

/* ============================================================================
* void read_bmp_16_noshift()
============================================================================ */
void read_bmp_16_noshift(uint16_t** img_raw, uint16_t sx, uint16_t sy,
                         char* fname) {
  FILE* f;
  int row, col;
  uint16_t* img;
  uint8_t* pin;
  uint16_t* pout;
  uint8_t* row_buf;
  uint8_t bmp_head[54];
  uint16_t sx_1, sy_1;

  f = fopen(fname, "rb");
  if (f == NULL) {
    printf("Error opening raw image %s\n", fname);
    exit(1);
  }

  if (fread(&bmp_head, 1, sizeof(bmp_head), f) != sizeof(bmp_head)) {
    printf("Error reading input image %s\n", fname);
    fclose(f);
    exit(1);
  }
  sx_1 = bmp_head[18] + ((uint16_t)bmp_head[19] << 8);
  sy_1 = bmp_head[22] + ((uint16_t)bmp_head[23] << 8);

  if (sx_1 != sx || sy_1 != sy) {
    printf(
        "Error: BMP size does not match specified size!\nBMP: %d %d\nExpected: "
        "%d %d",
        sx_1, sy_1, sx, sy);
    exit(1);
  }

  img = (uint16_t*)malloc(sx * sy * 3 * 2);
  if (img == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * sy * 3);
    exit(1);
  }

  row_buf = (uint8_t*)malloc(sx * 3);

  for (row = sy - 1; row >= 0; row--) {
    // read 1 rows
    fread(row_buf, 1, sx * 3, f);

    pin = row_buf;

    pout = img + 3 * row * sx;
    for (col = 0; col < sx; col++) {
      register uint16_t r, g, b;
      b = (uint16_t)(*pin++);
      g = (uint16_t)(*pin++);
      r = (uint16_t)(*pin++);
      *pout++ = r;
      *pout++ = g;
      *pout++ = b;
    }
  }

  *img_raw = img;
  fclose(f);
}

/* ============================================================================
* void read_bmp()
============================================================================ */
void read_bmp(uint16_t** img_rgb, uint16_t* size_x, uint16_t* size_y,
              char* fname) {
  FILE* f;
  int row, col;
  uint16_t* img;
  uint16_t sx;
  uint16_t sy;
  uint8_t* pin;
  uint16_t* pout;
  uint8_t* row_buf;
  uint8_t bmp_head[54];

  f = fopen(fname, "rb");
  if (f == NULL) {
    printf("Error opening image %s\n", fname);
    exit(1);
  }

  if (fread(&bmp_head, 1, sizeof(bmp_head), f) != sizeof(bmp_head)) {
    printf("Error reading input image %s\n", fname);
    fclose(f);
    exit(1);
  }
  sx = bmp_head[18] + ((uint16_t)bmp_head[19] << 8);
  sy = bmp_head[22] + ((uint16_t)bmp_head[23] << 8);

  img = (uint16_t*)malloc(sx * sy * 3 * 2);
  if (img == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * sy * 3);
    exit(1);
  }

  row_buf = (uint8_t*)malloc(sx * 3);
  if (row_buf == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * 3);
    free(img);
    exit(1);
  }

  for (row = sy - 1; row >= 0; row--) {
    // read 1 rows
    fread(row_buf, 1, sx * 3, f);

    pin = row_buf;

    pout = img + 3 * row * sx;
    for (col = 0; col < sx; col++) {
      register uint32_t r, g, b;
      b = (uint16_t)(*pin++) << 8;
      g = (uint16_t)(*pin++) << 8;
      r = (uint16_t)(*pin++) << 8;
      *pout++ = r;
      *pout++ = g;
      *pout++ = b;
    }
  }

  *img_rgb = img;
  *size_x = sx;
  *size_y = sy;

  free(row_buf);
  fclose(f);
}

/* ============================================================================
* void process_raw()
============================================================================ */

void process_raw(proc_in_t* proc_in, uint16_t sx, uint16_t sy, uint16_t ppln,
                 uint16_t* in_raw, uint16_t** out_rgb, uint8_t bits_per_pix) {
  int row, col;
  uint16_t* img;
  uint16_t* pimg;
  uint16_t *pin0, *pin1;
  uint32_t i;

  img = (uint16_t*)malloc(sx * sy * 3 * 2);
  if (img == NULL) {
    printf("Can not allocate memory for image %d bytes\n", sx * sy * 2);
    exit(1);
  }

  pimg = img;
  for (row = 0; row < sy / 2; row++) {
    pin0 = in_raw + ppln * 2 * row;
    pin1 = pin0 + ppln;

    for (col = 0; col < sx / 2; col++) {
      register uint32_t r, g0, g1, b;

      switch (proc_in->col_ptn) {
        case RAW_PTN_RGGB:
          r = (*pin0++);
          g0 = (*pin0++);
          g1 = (*pin1++);
          b = (*pin1++);
          if ((r & 0x8000) || (g0 & 0x8000) || (g1 & 0x8000) || (b & 0x8000)) {
            r = g0 = g1 = b = (1 << bits_per_pix) / 2;
          }
          break;

        case RAW_PTN_GRBG:
          g0 = (*pin0++);
          r = (*pin0++);
          b = (*pin1++);
          g1 = (*pin1++);
          if ((r & 0x8000) || (g0 & 0x8000) || (g1 & 0x8000) || (b & 0x8000)) {
            r = g0 = g1 = b = (1 << bits_per_pix) / 2;
          }
          break;
        case RAW_PTN_GBRG:
          g0 = (*pin0++);
          b = (*pin0++);
          r = (*pin1++);
          g1 = (*pin1++);
          if ((r & 0x8000) || (g0 & 0x8000) || (g1 & 0x8000) || (b & 0x8000)) {
            r = g0 = g1 = b = (1 << bits_per_pix) / 2;
          }
          break;
        case RAW_PTN_BGGR:
        default:
          b = (*pin0++);
          g0 = (*pin0++);
          g1 = (*pin1++);
          r = (*pin1++);
          if ((r & 0x8000) || (g0 & 0x8000) || (g1 & 0x8000) || (b & 0x8000)) {
            r = g0 = g1 = b = (1 << bits_per_pix) / 2;
          }
      }

      r <<= (16 - bits_per_pix);
      g0 <<= (16 - bits_per_pix);
      g1 <<= (16 - bits_per_pix);
      b <<= (16 - bits_per_pix);

      // write 4 pixels
      *pimg++ = r;
      *pimg++ = g0;
      *pimg++ = b;

      *pimg++ = r;
      *pimg++ = g0;
      *pimg++ = b;

      pimg += sx * 3 - 6;
      *pimg++ = r;
      *pimg++ = g1;
      *pimg++ = b;

      *pimg++ = r;
      *pimg++ = g1;
      *pimg++ = b;
      pimg -= sx * 3;
    }
    pimg += sx * 3;
  }

  // generate processed image
  pimg = img;
  for (i = 0; i < (uint32_t)sx * sy; i++) {
    int32_t r, g, b;
    int32_t r1, g1, b1;
    int32_t wb, ofst;

    // red
    wb = proc_in->k_awb_r;
    ofst = proc_in->pre_offsets[0] << 6;  // 10 bit
    r = (*pimg++);                        // convert to 10 bit
    r = ((uint32_t)r * wb + 128) / 256;
    r = r + ofst;
    if (r < 0) {
      r = 0;
    }
    if (r > 65535) {
      r = 65535;
    }

    // green
    wb = proc_in->k_awb_g;
    ofst = proc_in->pre_offsets[1] << 6;  // 10 bit
    g = (*pimg++);                        // convert to 10 bit
    g = ((uint32_t)g * wb + 128) / 256;
    g = g + ofst;
    if (g < 0) {
      g = 0;
    }
    if (g > 65535) {
      g = 65535;
    }

    // blue
    wb = proc_in->k_awb_b;
    ofst = proc_in->pre_offsets[2] << 6;  // 10 bit
    b = (*pimg++);                        // convert to 10 bit
    b = ((uint32_t)b * wb + 128) / 256;
    b = b + ofst;
    if (b < 0) {
      b = 0;
    }
    if (b > 65535) {
      b = 65535;
    }

    r1 = (proc_in->ccm[0] * r + proc_in->ccm[1] * g + proc_in->ccm[2] * b +
          128) /
             256 +
         proc_in->post_offsets[0];
    g1 = (proc_in->ccm[3] * r + proc_in->ccm[4] * g + proc_in->ccm[5] * b +
          128) /
             256 +
         proc_in->post_offsets[1];
    b1 = (proc_in->ccm[6] * r + proc_in->ccm[7] * g + proc_in->ccm[8] * b +
          128) /
             256 +
         proc_in->post_offsets[2];
    r1 += proc_in->post_offsets[0] << 6;
    g1 += proc_in->post_offsets[1] << 6;
    b1 += proc_in->post_offsets[2] << 6;

    if (r1 > 65535) r1 = 65535;
    if (g1 > 65535) g1 = 65535;
    if (b1 > 65535) b1 = 65535;
    if (r1 < 0) r1 = 0;
    if (g1 < 0) g1 = 0;
    if (b1 < 0) b1 = 0;

    // apply gamma
    {
      // proc_in->gamma is 8bit->10bit
      uint32_t slp;
      slp = 0;
      if ((r1 >> 8) < 255) {
        slp = proc_in->gamma[(r1 >> 8) + 1] - proc_in->gamma[(r1 >> 8)];
      }
      r1 = ((uint32_t)proc_in->gamma[(r1 >> 8)] << 6) +
           (slp << 6) * (r1 & ((1 << 8) - 1)) / (1 << 8);
      slp = 0;
      if ((g1 >> 8) < 255) {
        slp = proc_in->gamma[(g1 >> 8) + 1] - proc_in->gamma[(g1 >> 8)];
      }
      g1 = ((uint32_t)proc_in->gamma[(g1 >> 8)] << 6) +
           (slp << 6) * (g1 & ((1 << 8) - 1)) / (1 << 8);
      slp = 0;
      if ((b1 >> 8) < 255) {
        slp = proc_in->gamma[(b1 >> 8) + 1] - proc_in->gamma[(b1 >> 8)];
      }
      b1 = ((uint32_t)proc_in->gamma[(b1 >> 8)] << 6) +
           (slp << 6) * (b1 & ((1 << 8) - 1)) / (1 << 8);
    }

    // output 16bit MSB allligned pixels
    pimg -= 3;
    *pimg++ = r1;
    *pimg++ = g1;
    *pimg++ = b1;
  }

  *out_rgb = img;
}

/* ============================================================================
* int write_bmp()
============================================================================ */
int write_bmp(uint16_t* img_rgb, uint16_t sx, uint16_t sy, char* fname,
              char* fname_suff) {
  FILE* out;
  int i, j;
  uint32_t file_size;
  int bmp_size_x;
  char out_file_name[256];
  char* temp_ptr;
  uint16_t* prow;
  uint8_t* row_buf;
  uint8_t* prow_out;
  // #define KA (0.000024)//(0.000048)
  //  sx = 2304;//test
  //  sy = 256;//test

  bmp_size_x = sx & ~0x3;  // must be multiple of 4
  // generate file name
  strcpy(out_file_name, fname);
  temp_ptr = strrchr(out_file_name, '.');
  if (temp_ptr) {
    *temp_ptr = '\0';
  }
  strcat(out_file_name, fname_suff);

  row_buf = (uint8_t*)malloc(bmp_size_x * 3);
  if (row_buf == NULL) {
    printf("Can not allocate memory for image %d bytes\n", bmp_size_x * 3);
    exit(1);
  }

  out = fopen(out_file_name, "wb");
  if (out == NULL) {
    printf("Error opening oputput file %s\n", out_file_name);
    exit(1);
  }

  // prepare/write file header

  file_size = (bmp_size_x * sy * 3 + 54 + 2);
  head[2] = file_size & 0xff;
  head[3] = (file_size >> 8) & 0xff;
  head[4] = (file_size >> 16) & 0xff;

  head[18] = bmp_size_x & 0xff;
  head[19] = (bmp_size_x >> 8) & 0xff;

  head[22] = sy & 0xff;
  head[23] = (sy >> 8) & 0xff;

  fwrite(head, 1, 54, out);

  for (i = sy - 1; i >= 0; i--) {
    prow = img_rgb + 3 * i * sx;
    prow_out = row_buf;
    for (j = 0; j < bmp_size_x; j++) {
      uint16_t r, g, b;
      r = (*prow++) >> 8;
      g = (*prow++) >> 8;
      b = (*prow++) >> 8;
      *prow_out++ = b;
      *prow_out++ = g;
      *prow_out++ = r;
    }
    fwrite(row_buf, 1, bmp_size_x * 3, out);
  }
  /*
  for (i =sy-1; i>=0; i--)
  {
      prow_out = row_buf;
      for (j =0; j<bmp_size_x; j++)
      {
          *prow_out++ = KA*j*j + ((255.0/2304.0)-2304*KA)*j;
          *prow_out++ = KA*j*j + ((255.0/2304.0)-2304*KA)*j;
          *prow_out++ = KA*j*j + ((255.0/2304.0)-2304*KA)*j;
      }
      fwrite(row_buf, 1, bmp_size_x*3, out);
  }
  */
  fclose(out);
  return 0;
}
/* ============================================================================
* void write_raw()
============================================================================ */
void write_raw(uint16_t* img_raw, uint16_t sx, uint16_t sy, char* fname) {
  FILE* f;
  uint32_t pixels_written;

  f = fopen(fname, "wb");
  if (f == NULL) {
    printf("Error opening raw image %s\n", fname);
    exit(1);
  }

  pixels_written = fwrite(img_raw, 2, sx * sy, f);

  if (pixels_written != sx * sy) {
    printf("Error writing raw image %s\n", fname);
    exit(1);
  }

  fclose(f);
}
