#include "fuse.h"

#include <string.h>

#include <cstdint>

//-----------------------------------------------------------------------------
//-------------------- DEFS ---------------------------------------------------
//-----------------------------------------------------------------------------
static const unsigned short RECIPROCAL_8[256] = {
    65535, 65535, 32768, 21845, 16384, 13107, 10923, 9362, 8192, 7282, 6554,
    5958,  5461,  5041,  4681,  4369,  4096,  3855,  3641, 3449, 3277, 3121,
    2979,  2849,  2731,  2621,  2521,  2427,  2341,  2260, 2185, 2114, 2048,
    1986,  1928,  1872,  1820,  1771,  1725,  1680,  1638, 1598, 1560, 1524,
    1489,  1456,  1425,  1394,  1365,  1337,  1311,  1285, 1260, 1237, 1214,
    1192,  1170,  1150,  1130,  1111,  1092,  1074,  1057, 1040, 1024, 1008,
    993,   978,   964,   950,   936,   923,   910,   898,  886,  874,  862,
    851,   840,   830,   819,   809,   799,   790,   780,  771,  762,  753,
    745,   736,   728,   720,   712,   705,   697,   690,  683,  676,  669,
    662,   655,   649,   643,   636,   630,   624,   618,  612,  607,  601,
    596,   590,   585,   580,   575,   570,   565,   560,  555,  551,  546,
    542,   537,   533,   529,   524,   520,   516,   512,  508,  504,  500,
    496,   493,   489,   485,   482,   478,   475,   471,  468,  465,  462,
    458,   455,   452,   449,   446,   443,   440,   437,  434,  431,  428,
    426,   423,   420,   417,   415,   412,   410,   407,  405,  402,  400,
    397,   395,   392,   390,   388,   386,   383,   381,  379,  377,  374,
    372,   370,   368,   366,   364,   362,   360,   358,  356,  354,  352,
    350,   349,   347,   345,   343,   341,   340,   338,  336,  334,  333,
    331,   329,   328,   326,   324,   323,   321,   320,  318,  317,  315,
    314,   312,   311,   309,   308,   306,   305,   303,  302,  301,  299,
    298,   297,   295,   294,   293,   291,   290,   289,  287,  286,  285,
    284,   282,   281,   280,   279,   278,   277,   275,  274,  273,  272,
    271,   270,   269,   267,   266,   265,   264,   263,  262,  261,  260,
    259,   258,   257};

#define abs(a) (((a) > 0) ? (a) : -(a))
#define CLIP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
//-----------------------------------------------------------------------------
//-------------------- routines -----------------------------------------------
//-----------------------------------------------------------------------------

typedef struct {
  uint8_t y_weight[256];
  uint32_t percent_min;  // 1/1024
  uint32_t percent_max;  // 1/1024
  uint8_t max_sub;
  uint8_t max_target;
  uint16_t max_expand;

} mfhdr_tune_data_t;
void blankk() {}
mfhdr_tune_data_t mfhdr_tune_data = {
    {
#include "mfhdr_y_weight.dat"
    },    // uint8_t y_weight[256];
    1,    // uint32_t percent_min;// 1/1024
    1,    // uint32_t percent_max;// 1/1024
    20,   // uint8_t  max_sub;
    200,  // uint8_t  max_target;
    512,  // uint16_t max_expand; //U16Q8
};

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

int test_mfhdr_process(unsigned short** images, short imageCount,
                       unsigned short* out, short width, short height) {
  uint32_t hist[256];
  uint16_t* pimgs[5];
  uint16_t* img_o = out;
  int row, col;
  int i;
  uint32_t sum;
  uint32_t expand;
  int32_t blk_sub;

  // memset(ctx->hist,0,sizeof(ctx->hist));
  memset(hist, 0, sizeof(hist));

  for (i = 0; i < imageCount; i++) {
    pimgs[i] = images[i];
    if (pimgs[i] == 0) {
      return (-1);
    }
  }

  for (row = 0; row < height; row++) {
    for (col = 0; col < width; col++) {
      int16_t oy;
      int32_t ou;
      uint16_t sum_wc;

      // determine weights for blending c-s
      oy = 0;
      ou = 0;
      sum_wc = 0;
      for (i = 0; i < imageCount; i++) {
        uint16_t wc;
        int16_t y, u;
        y = (*(pimgs[i])) >> 8;
        u = (int16_t)((*(pimgs[i])) & 0x00FF) - 128;
        pimgs[i]++;
        wc = mfhdr_tune_data.y_weight[y];
        oy += y;
        ou += wc * u;
        sum_wc += wc;
      }
      // oy /= imageCount;
      oy = ((uint32_t)oy * RECIPROCAL_8[imageCount]) / 65536;

      if (sum_wc >= 256) {
        sum_wc /= 8;
        ou /= 8;
      }
      ou = ((uint32_t)ou * RECIPROCAL_8[sum_wc]) / 65536;

      hist[(uint8_t)oy]++;

      *img_o++ = ((uint16_t)oy << 8) + (uint16_t)(ou + 128);
    }
  }
  sum = 0;
  for (i = 0; i < 256; i++) {
    sum += hist[i];
    if (sum >= width * height * mfhdr_tune_data.percent_min / 1024) {
      break;
    }
  }
  blk_sub = i;
  if (blk_sub > mfhdr_tune_data.max_sub) {
    blk_sub = 2 * (int32_t)mfhdr_tune_data.max_sub - blk_sub;
  }
  if (blk_sub < 0) blk_sub = 0;

  sum = 0;
  for (i = 255; i > 0; i--) {
    sum += hist[i];
    if (sum >= (width * height * mfhdr_tune_data.percent_max) / 1024) {
      break;
    }
  }

  if (i == 0) {
    expand = 256;
  } else {
    expand = ((uint32_t)mfhdr_tune_data.max_target * 256) / i;
  }
  if (expand > mfhdr_tune_data.max_expand) {
    expand = mfhdr_tune_data.max_expand;
  }
  if (expand < 256) {
    expand = 256;
  }
  expand = (expand * 256) / (255 - blk_sub);

  img_o = out;
  for (row = 0; row < height; row++) {
    for (col = 0; col < width; col++) {
      int16_t oy, ou, ov;
      int16_t oyn;

      oy = (*img_o) >> 8;
      ou = (*img_o) & 0x00FF;
      ou -= 128;

      oyn = ((oy - blk_sub) * expand) / 256;
      oyn = CLIP(oyn, 0, 255);
      //          ou = (oyn * ou + 128*oy + oy/2) / oy;
      ou = (((int32_t)oyn * ou) * RECIPROCAL_8[oy]) / 65536;
      ou += 128;
      ou = CLIP(ou, 0, 255);

      *img_o++ = ((uint16_t)oyn << 8) + (uint16_t)ou;
    }
    // img_o += (din->img_ppln - din->img_sx);
  }
  return 0;
}
