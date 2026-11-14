#include "rhdr.h"

#include <assert.h>
#include <limits.h>
#include <math.h>

#include "rhdr_prv.h"
//-----------------------------------------------------------------------------
//-------------------- DEFS ---------------------------------------------------
//-----------------------------------------------------------------------------
void blank(void);  // test

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))
/*
static const uint16_t RECIPROCAL_8[256] =
{
65535	,65535	,32768	,21845	,16384	,13107
,10923	,9362	,8192	,7282	,6554	,5958 ,5461	,5041	,4681	,4369
, 4096	,3855	,3641	,3449	,3277	,3121	,2979
,2849	,2731	,2621	,2521	,2427	,2341	,2260	,2185	,2114	, 2048
,1986	,1928	,1872	,1820	,1771	,1725
,1680	,1638	,1598	,1560	,1524	,1489	,1456	,1425	,1394	, 1365
,1337	,1311	,1285	,1260	,1237	,1214
,1192	,1170	,1150	,1130	,1111	,1092	,1074	,1057	,1040	, 1024
,1008	,993	,978	,964	,950	,936	,923
,910	,898	,886	,874	,862	,851	,840	,830	, 819	,809
,799	,790	,780	,771	,762	,753	,745
,736	,728	,720	,712	,705	,697	,690	, 683	,676
,669	,662	,655	,649	,643	,636	,630
,624	,618	,612	,607	,601	,596	,590	, 585	,580
,575	,570	,565	,560	,555	,551	,546
,542	,537	,533	,529	,524	,520	,516	, 512	,508
,504	,500	,496	,493	,489	,485	,482
,478	,475	,471	,468	,465	,462	,458	, 455	,452
,449	,446	,443	,440	,437	,434	,431
,428	,426	,423	,420	,417	,415	,412	, 410	,407
,405	,402	,400	,397	,395	,392	,390
,388	,386	,383	,381	,379	,377	,374	, 372	,370
,368	,366	,364	,362	,360	,358	,356
,354	,352	,350	,349	,347	,345	,343	, 341	,340
,338	,336	,334	,333	,331	,329	,328
,326	,324	,323	,321	,320	,318	,317	, 315	,314
,312	,311	,309	,308	,306	,305	,303
,302	,301	,299	,298	,297	,295	,294	, 293	,291
,290	,289	,287	,286	,285	,284	,282
,281	,280	,279	,278	,277	,275	,274	, 273	,272
,271	,270	,269	,267	,266	,265	,264
,263	,262	,261	,260	,259	,258	,257
};
*/

uint8_t col_ptn_mapping[RHDR_RAW_COLPTN_COUNT] = {
    0,  // RHDR_RAW_RGGB
    0,  // RHDR_RAW_BGGR
    1,  // RHDR_RAW_GRBG
    1,  // RHDR_RAW_GBRG
};
#define abs(a) (((a) > 0) ? (a) : -(a))
#define CLIP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//-----------------------------------------------------------------------------
//-------------------- DEBUG -----------------------------------------------
//-----------------------------------------------------------------------------

#define DEBUG_LEVEL 2
/*
   Debug levels:
   0 - None
   1 - Before validate_similarities
   2 - After validate_similarities
   3 - Weights -- NOT IN THIS BRANCH
   4 - Activity Masks -- NOT IN THIS BRANCH

   10 - Images after preprocessing (reverse gamma & stuff)

   Debug level accounts only for debug images.
   Don't define DEBUG_LEVEL to disable all
   debug calculations.
*/

#ifdef DEBUG_LEVEL

int debug_pixels[16][2];
int debug_pixels_count;
char debug_marks_file[] = "marks.txt";
char debug_dump_file[] = "dump.txt";
char debug_imgs_dump_prefix[] = "dbg_";

uint16_t* pDebugImgs[RHDR_MAX_IMGS];

void mallocDebugImgs(int imgs, int rows, int cols) {
  int index;
  for (index = 0; index < imgs; ++index)
    pDebugImgs[index] = (uint16_t*)malloc(rows * cols * 3 * sizeof(uint16_t));
}

void freeDebugImgs(int imgs) {
  int index;
  for (index = 0; index < imgs; ++index) free(pDebugImgs[index]);
}

#include "proc_img.h"
void dumpDebugImgs(int imgs, int rows, int cols) {
  int index;
  char suff[8];
  for (index = 0; index < imgs; ++index) {
    sprintf(suff, "%d.bmp", index);
    write_bmp(pDebugImgs[index], cols, rows, debug_imgs_dump_prefix, suff);
  }
}

void read_debug_pixels() {
  FILE* fin;
  int index;
  int x, y;
  fin = fopen(debug_marks_file, "r");
  if (!fin) {
    debug_pixels_count = 0;
    return;
  }

  fscanf(fin, "%d", &debug_pixels_count);
  for (index = 0; index < debug_pixels_count; ++index) {
    fscanf(fin, "%d%d", &x, &y);
    debug_pixels[index][0] = x;
    debug_pixels[index][1] = y;
  }

  fclose(fin);
}

int is_debug_pixel(int col, int row) {
  int index;

  for (index = 0; index < debug_pixels_count; ++index)
    if (col / 3 == debug_pixels[index][0] && row == debug_pixels[index][1])
      return 1;

  return 0;
}

#endif /* defined DEBUG_LEVEL */

//-----------------------------------------------------------------------------
//-------------------- routines -----------------------------------------------
//-----------------------------------------------------------------------------
static void validate_similatrities(rhdr_context_t* ctx);
static void prune_similarities(rhdr_context_t* ctx);
static void expand_non_similarities(rhdr_context_t* ctx);
static float neighbors_factor(rhdr_context_t* ctx, int img, int row, int col);
static void ave_11_11(rhdr_context_t* ctx, int img, uint16_t* pimg,
                      uint16_t* pref_img, int row, int col, float* out_average,
                      float* out_count_factor);
static int getChannel(int y, int x);

static int8_t mult2ev(uint16_t mult) {
  if (mult == 0) {
    return 0;
  }
  return (int)(log(2.0));
  //    return (int8_t)(0.5 + 10*log((double)mult/256) / log(2.0));
}

static void reverse_gamma(uint16_t* gamma_orig, uint16_t* gamma_out) {
  // Reverse gamma from 256x8-bit to 256x12-bit
  uint16_t* p;
  uint16_t index;
  int16_t step;
  uint16_t counter;
  uint16_t i;

  p = gamma_out;
  counter = 0;

  *p++ = counter;
  counter += 16;

  for (index = 1; index < 256; ++index) {
    step = gamma_orig[index] - gamma_orig[index - 1];
    for (i = 0; i < step; ++i) *p++ = counter + (16 * i) / step;

    counter += 16;
  }
}

int32_t test_rhdr_create(void** context) {
  *context = (rhdr_context_t*)malloc(sizeof(rhdr_context_t));

  if (*context == NULL) {
    return 1;
  }

  return 0;
}

int32_t test_rhdr_process(void* context, rhdr_in_data_t* din) {
  rhdr_context_t* ctx;
  uint16_t* pimgs[RHDR_MAX_IMGS];
  uint16_t* pimgs_orig[RHDR_MAX_IMGS];
  uint16_t pix_dark_snr_thr[RHDR_MAX_IMGS];
  uint16_t ns_base[RHDR_MAX_IMGS];
  uint16_t ns_slp[RHDR_MAX_IMGS];
  uint8_t used[RHDR_MAX_IMGS];
  uint32_t unknown_overexp;
  uint32_t unknown_dark;
  uint8_t* pimgs_mask;
  uint16_t rev_gamma[256];

  uint16_t* img_o = din->out_rgb;
  int row, col;
  int i, j;
  uint64_t min_texp_all;

/*** test { */
#ifdef DEBUG_LEVEL
  uint16_t* pDbgImgs[RHDR_MAX_IMGS];
  FILE* debugfile = fopen(debug_dump_file, "w");
  read_debug_pixels();
  mallocDebugImgs(din->img_count, din->img_sy, din->img_sx);
  for (i = 0; i < din->img_count; ++i) pDbgImgs[i] = pDebugImgs[i];
#endif
  /*** } test */

  if ((context == NULL) || (din == NULL)) {
    // no input data or context
    return 1;
  }
  ctx = (rhdr_context_t*)context;
  /* clear histograms and the whole context */
  memset(ctx, 0, sizeof(rhdr_context_t));

  ctx->din = din;

  if (din->enable == 0) {
    // if disabled, take the middle image, sibtract data pedestal, restore
    // dynamic range and convert to output bits-per-pix
    din->img_count = 1;
    pimgs[0] = din->ptr_imgs[din->img_count / 2];
  }
  // allocate correspondece mask buffer
  ctx->pimgs_mask =
      (uint8_t*)malloc((din->img_ppln) * (din->img_sy + 1) *
                       3);  // reserve one extra row for expand_non_similarities
  if (ctx->pimgs_mask == NULL) {
    // can not allocate memory
    return 2;
  }

  ctx->ppruned_mask = (uint8_t*)malloc((din->img_ppln) * (din->img_sy) * 3);
  if (ctx->ppruned_mask == NULL) {
    // can not allocate memory
    free(ctx->pimgs_mask);
    return 2;
  }

  for (j = 0; j < din->img_count; ++j) used[j] = 0;

  // sort images from darkest to brightest
  // find brightest and darkest image
  for (j = 0; j < din->img_count; j++) {
    uint64_t min_texp;
    int32_t tmp;
    uint8_t darkest;

    min_texp = (uint64_t)ULLONG_MAX;
    for (i = 0; i < din->img_count; i++) {
      if (min_texp > ((uint64_t)din->img_exps_us[i] * din->img_gains[i]) &&
          used[i] == 0) {
        min_texp = ((uint64_t)din->img_exps_us[i] * din->img_gains[i]);
        darkest = i;
      }
    }
    pimgs[j] = din->ptr_imgs[darkest];
    used[darkest] = 1;
    ns_base[j] =
        din->tune_data->noise_vs_gain[mult2ev(din->img_gains[darkest])].base
        << (RHDR_OUT_BITS_PER_PIX - 8);
    ns_slp[j] =
        din->tune_data->noise_vs_gain[mult2ev(din->img_gains[darkest])].slope;
    // calc dark SNR threshold
    // noise = (pix*slp + base)/256, SNR = pix/noise
    //  pix = (base*SNR)/(256 - slp*SNR)
    tmp = 256 - din->tune_data->dark_snr_thr * ns_slp[j];
    if (tmp <= 0) {
      pix_dark_snr_thr[j] = (1 << RHDR_OUT_BITS_PER_PIX) / 2;  // default value
    } else {
      pix_dark_snr_thr[j] = (din->tune_data->dark_snr_thr * ns_base[j]) /
                            (256 - din->tune_data->dark_snr_thr * ns_slp[j]);
    }
    if (j == 0) {
      min_texp_all = min_texp;
    }

    if (min_texp_all <= 0x0000FFFFFFFFFFFF) {
      ctx->exp_ratio[j] = (min_texp_all * 0x10000) / min_texp;
    } else {
      ctx->exp_ratio[j] = min_texp_all / (min_texp / 0x10000);
    }
    if (pimgs[j] == NULL) {
      // missing input image
      return (-1);
    }
  }

  for (j = 0; j < din->img_count; ++j) pimgs_orig[j] = pimgs[j];

  // #########################################################################
  //  Copy/check/modify tunning data
  // #########################################################################
  //  prepare weight tables
  for (i = 0; i < RHDR_WEIGHT_TABLE_COUNT; i++) {
    memcpy(ctx->p_weight_tbl[i], din->tune_data->y_weight,
           sizeof(ctx->p_weight_tbl[0]));
    if (i == 0) {
      // this table will be used for the darkest matching image - set its
      // weights around 255 to 1
      uint8_t* ptbl;
      ptbl = &ctx->p_weight_tbl[i][255];
      while ((*ptbl == 0) && (ptbl > &ctx->p_weight_tbl[i][0])) {
        *ptbl-- = 1;
      }
    }
    if (i == (RHDR_WEIGHT_TABLE_COUNT - 1)) {
      // this table will be used for the brightest matching image - set its
      // weights around 0 to 1
      uint8_t* ptbl;
      ptbl = &ctx->p_weight_tbl[i][0];
      while ((*ptbl == 0) && (ptbl < &ctx->p_weight_tbl[i][255])) {
        *ptbl++ = 1;
      }
    }
  }

  // #########################################################################
  //       apply DP, WB, Reverse Gamma etc.
  // #########################################################################
  reverse_gamma(ctx->din->in_gamma, rev_gamma);

  for (i = 0; i < din->img_count; ++i) {
    uint16_t* pimg0 = pimgs[i];
    for (row = 0; row < din->img_sy; row++) {
      for (col = 0; col < din->img_sx * 3; col++) {
        int32_t p0;
        // read pixel
        p0 = *pimg0;

        // apply gamma (this also converts the image to 12 bpp)
        p0 = rev_gamma[p0 & 0xff];

        // ensure boundaries
        if (p0 < 0) {
          p0 = 0;
        }
        if (p0 > ((1 << RHDR_OUT_BITS_PER_PIX) - 1)) {
          p0 = ((1 << RHDR_OUT_BITS_PER_PIX) - 1);
        }
        *pimg0++ = p0;

/*** test { */
#if DEBUG_LEVEL == 10
        *pDbgImgs[i]++ = p0 << 4;
#endif
        /*** } test */
      }
      pimg0 += (din->img_ppln - din->img_sx) * 3;
    }
  }

  // #########################################################################
  //       match raws
  // #########################################################################

  // #########################################################################
  //       find moving objects
  // #########################################################################

  // mark similar pixels to ref image in all images
  pimgs_mask = ctx->pimgs_mask;
  for (row = 0; row < din->img_sy; row++) {
    for (col = 0; col < din->img_sx * 3; col++) {
      int32_t p[RHDR_MAX_IMGS];
      int32_t p_min[RHDR_MAX_IMGS];
      int32_t p_max[RHDR_MAX_IMGS];
      int32_t noise[RHDR_MAX_IMGS];
      uint8_t imgs_mask, mask;

/*** test { */
#ifdef DEBUG_LEVEL
      if (is_debug_pixel(col, row)) {
        blank();
      }
#endif
      /*** } test */

      for (i = 0; i < din->img_count; i++) {
        // read pixel, subtract data pedestal, retrieve dynamic range, convert
        // to output bits per pix
        p[i] = *(pimgs[i])++;

        if (p[i] >= RHDR_SAT_LEVEL_DIF_DIFF) {
          p_min[i] = (RHDR_SAT_LEVEL_DIF_DIFF * ctx->exp_ratio[i]) / 0x10000;
          p_max[i] = (1 << RHDR_OUT_BITS_PER_PIX) - 1;
        } else {
          noise[i] = (p[i] * ns_slp[i] + ns_base[i]) / 256;
          if (p[i] > noise[i])
            p_min[i] = ((p[i] - noise[i]) * ctx->exp_ratio[i]) / 0x10000;
          else
            p_min[i] = 0;
          p_max[i] = ((p[i] + noise[i]) * ctx->exp_ratio[i]) / 0x10000;
        }
      }
      imgs_mask = 0;
      for (i = 0, mask = 1; i < din->img_count; i++, mask <<= 1) {
        if ((p_min[i] <= p_max[din->ref_image]) &&
            (p_max[i] >= p_min[din->ref_image])) {
          // the pixel is similar to the ref image
          imgs_mask |= mask;
        }
      }
      *pimgs_mask++ = imgs_mask;

/*** test { */
#ifdef DEBUG_LEVEL
#if DEBUG_LEVEL == 1
      for (i = 0; i < din->img_count; ++i)
        if ((imgs_mask >> i) & 1) {
          *pDbgImgs[i]++ = 0x3fff * 4;
        } else {
          *pDbgImgs[i]++ = 0;
        }
#endif

      if (is_debug_pixel(col, row)) {
        int index;
        /* dump pixel info */
        fprintf(debugfile, "Pixel: X %d, Y %d, Color %d\n", col, row,
                getChannel(row, col));
        fprintf(debugfile, "Value\tNoise\tMin\tMax\tMask\n");
        for (index = 0; index < din->img_count; ++index)
          fprintf(debugfile, "%c%d\t%d\t%d\t%d\t%d\n",
                  (index == din->ref_image ? '*' : ' '), p[index], noise[index],
                  p_min[index], p_max[index], (imgs_mask >> index) & 1);
        fputs("\n\n", debugfile);
      }
#endif /* defined DEBUG_LEVEL */
       /*** } test */
    }
    pimgs_mask += (din->img_ppln - din->img_sx) * 3;
    for (i = 0; i < din->img_count; i++) {
      pimgs[i] += (din->img_ppln - din->img_sx) * 3;
    }
  }
  // turn pimgs pointers back
  for (i = 0; i < din->img_count; i++) {
    pimgs[i] = pimgs_orig[i];
  }

  // #########################################################################
  //       blend images
  // #########################################################################

  /* Note:
   * - 'prune_similarities' takes the mask from 'ctx->pimgs_mask'
   *		and outputs to 'ctx->ppruned_mask'
   * - 'validate_similarities' takes the mask from 'ctx->ppruned_mask'
   *		and outputs in-place
   * - 'expand_non_similarities' takes the mask from 'ctx->ppruned_mask'
   *		and outputs to 'ctx->pimgs_mask'

   */
  // check the comments inside each function for information
  prune_similarities(ctx);
  validate_similatrities(ctx);
  expand_non_similarities(ctx);

/*** test { */
#ifdef DEBUG_LEVEL
  fprintf(debugfile, "----------------------------------------------\n\n");
#endif
  /*** } test */

  // if a pixel is overexposed in ref image, then leave only the corresponding
  // pixel in the darkest image which is marked "similar". All other pixels
  //(in brighter images) are not known - even if they are similar to the ref
  // image (overexposed), they may not be similar to the darkest similar image.
  // The darkest similar image is cosidered a reference image for this pixels.
  // The above could be said for dark pixels in ref image also - they have
  // corresponding brightest similar image, which is considered reference for
  // this pixel. Other (darker images are not known) whether they are similar
  // to this new ref image.
  // This check can be done in the final fising loop.

  unknown_overexp = 0;
  unknown_dark = 0;
  pimgs_mask = ctx->pimgs_mask;

  for (row = 0; row < din->img_sy; row++) {
    for (col = 0; col < din->img_sx * 3; col++) {
      int ref_p;
      int ref_weight;
      float op;
      uint8_t mask_similar;

/*** test { */
#ifdef DEBUG_LEVEL
      if (is_debug_pixel(col, row)) {
        blank();
      }
#endif
      /*** } test */

      mask_similar = *pimgs_mask;
      ref_p = *(pimgs[din->ref_image]);
      ref_weight = ctx->p_weight_tbl[1][ref_p >> (RHDR_OUT_BITS_PER_PIX - 8)];

      if (ref_weight == 255) {
        // the pixel is properly exposed in reference image, use it
        op = (float)(ref_p * ctx->exp_ratio[din->ref_image]) / (float)0x10000;

/*** test { */
#ifdef DEBUG_LEVEL
        if (is_debug_pixel(col, row)) {
          fprintf(debugfile, "Pixel: X %d, Y %d, Color %d\n", col, row,
                  getChannel(row, col));
          fprintf(debugfile, "Properly exposed; Final value: %d\n\n\n",
                  (uint16_t)op);
        }
#endif
        /*** } test */
      } else {
        // the pixel is not properly exposed in ref image
        // check if it is over or underexposed
        int p[RHDR_MAX_IMGS];
        float wp[RHDR_MAX_IMGS];

        uint8_t cnt_similar = 0;
        float sum_wp, remaining_weight;

        float average = -1, count_factor;

        // clear arrays
        for (i = 0; i < RHDR_MAX_IMGS; i++) {
          p[i] = 0;
          wp[i] = 0;
        }
        sum_wp = 0;

        remaining_weight = 255 - ref_weight;

        if (ref_p >> (RHDR_OUT_BITS_PER_PIX - 8) >= 128) {
          // OVEREXPOSED
          // read pixel values from darkest to ref image
          for (i = 0; i < din->ref_image; ++i) {
            p[i] = *pimgs[i];
            wp[i] = ctx->p_weight_tbl[cnt_similar ? 1 : 0]
                                     [p[i] >> (RHDR_OUT_BITS_PER_PIX - 8)];
            wp[i] *= (mask_similar >> i) & 1;
            cnt_similar += (wp[i] > 0);
            sum_wp += wp[i];
          }
        } else {
          // UNDEREXPOSED
          // read pixel values from brightest image to ref
          for (i = din->img_count - 1; i > din->ref_image; --i) {
            p[i] = *pimgs[i];
            wp[i] = ctx->p_weight_tbl[cnt_similar ? 1 : 2]
                                     [p[i] >> (RHDR_OUT_BITS_PER_PIX - 8)];
            wp[i] *= (mask_similar >> i) & 1;
            cnt_similar += (wp[i] > 0);
            sum_wp += wp[i];
          }
        }

        p[din->ref_image] = ref_p;
        wp[din->ref_image] = ref_weight;

        if (sum_wp > 0 && cnt_similar >= 2)
          for (j = 0; j < din->img_count; j++)
            if (j != din->ref_image)
              wp[j] = (remaining_weight * wp[j]) / sum_wp;

        /*for(i=0; i<din->img_count; i++)
                if(wp[i] > 0) {
                        float factor = neighbors_factor(ctx, i, row, col);
                        wp[i] *= factor;
                }*/

        sum_wp = 0;
        for (i = 0; i < din->img_count; i++) {
          p[i] = (p[i] * ctx->exp_ratio[i]) / 0x10000;
          sum_wp += wp[i];
        }

        if (cnt_similar == 0 && (ref_p >> (RHDR_OUT_BITS_PER_PIX - 8) >= 128)) {
          // if the reference image is overexposed, but the
          // other images are not similar in this region,
          // try to get an estimate about the value of the pixel,
          // based on the surrounding, similar, pixels
          int img = (ref_p >> (RHDR_OUT_BITS_PER_PIX - 8) >= 128)
                        ? 0
                        : (din->ref_image - 1);
          ave_11_11(ctx, img, pimgs_orig[img], pimgs_orig[din->ref_image], row,
                    col, &average, &count_factor);
          p[img] = (int)average;
          p[img] = (p[img] * ctx->exp_ratio[img]) / 0x10000;
          wp[img] = count_factor * 100;
          wp[din->ref_image] += 100 - wp[img];
          sum_wp += 100;
        }

        if (sum_wp == 0) {
          wp[din->ref_image] = 1;
          sum_wp = 1;
        }

        op = wp[0] * p[0] + wp[1] * p[1] + wp[2] * p[2] + wp[3] * p[3] +
             wp[4] * p[4];
        op /= sum_wp;

/*** test {*/
#ifdef DEBUG_LEVEL
        if (is_debug_pixel(col, row)) {
          fprintf(debugfile, "Pixel: X %d, Y %d, Color %d\n", col, row,
                  getChannel(row, col));
          fprintf(debugfile, "Status: %s\n",
                  ((ref_p >> (RHDR_OUT_BITS_PER_PIX - 8) >= 128) ? "OVEREXPOSED"
                                                                 : "TOO DARK"));

          fprintf(debugfile, "Value\tWeight\tMask\n");
          for (j = 0; j < din->img_count; ++j)
            fprintf(debugfile, "%c%d\t%.3f\t%d\n",
                    (j == din->ref_image ? '*' : ' '), p[j], wp[j],
                    (mask_similar >> j) & 1);

          if (average >= 0)
            fprintf(debugfile, "AVERAGED: value = %.3f, factor = %.3f\n",
                    average, count_factor);

          fprintf(debugfile, "Final Value: %d\n\n\n", (uint32_t)op);
        }
#endif
        /*** } test*/
      } /* end of 'else' */

/*** test { */
#if DEBUG_LEVEL == 2
      for (i = 0; i < din->img_count; ++i)
        if ((mask_similar >> i) & 1) {
          *pDbgImgs[i]++ = 0x3fff * 4;
        } else {
          *pDbgImgs[i]++ = 0;
        }
#endif

      /*** } test */

      // write output pixel
      *img_o++ = (uint16_t)op;

      // advance pointers
      pimgs_mask++;
      for (i = 0; i < din->img_count; ++i) ++(pimgs[i]);

      // printf("r%d c%d\n", row, col);

    } /* end of column loop */

    pimgs_mask += (din->img_ppln - din->img_sx) * 3;
    for (i = 0; i < din->img_count; i++) {
      pimgs[i] += (din->img_ppln - din->img_sx) * 3;
    }
    img_o += (din->img_ppln - din->img_sx) * 3;
  } /* end of row loop */

  ctx->info.unknown_overexp = unknown_overexp;
  ctx->info.unknown_dark = unknown_dark;

/*** test { */
#ifdef DEBUG_LEVEL
#if DEBUG_LEVEL > 0
  dumpDebugImgs(din->img_count, din->img_sy, din->img_sx);
#endif
  fclose(debugfile);
  freeDebugImgs(din->img_count);
#endif
  /*** } test */

  return 0;
}

int32_t test_rhdr_delete(void* context) {
  if (context == NULL) {
    return 1;
  }
  free(context);
  return 0;
}

static void validate_similatrities(rhdr_context_t* ctx) {
  // Check RGB triads for similarity. A whole pixel should be marked
  // as similar or non-similar, not only the single colors of the pixel.
  // A pixel will be marked as similar if all three colors are similar.
  uint8_t* src;
  int16_t row, col;

  for (row = 0; row < ctx->din->img_sy; ++row) {
    src = ctx->ppruned_mask + (row * ctx->din->img_ppln * 3);
    for (col = 0; col < ctx->din->img_sx; ++col) {
      src[0] = src[1] = src[2] = src[0] & src[1] & src[2];
      src += 3;
    }
  }
}

static void prune_similarities(rhdr_context_t* ctx) {
  // prune_similarities: check that each color has at least five same-color
  // neighbors that match. Also if a non-similar pixel has all its same-color
  // neighbors marked as similar, mark it as similar as well.

  uint8_t* src;
  uint8_t* dst;
  int16_t row, col;
  int i, j;
  int index;
  int8_t color_count[3];  // color counts for R(0), G(1) and B(2)
  uint8_t* out;

  // also, we don't want to check first&last row&column,
  // because they don't have enough neighbours to be certain
  memcpy(ctx->ppruned_mask, ctx->pimgs_mask, ctx->din->img_ppln * 3);
  memcpy(ctx->ppruned_mask + (ctx->din->img_sy - 1) * ctx->din->img_ppln * 3,
         ctx->pimgs_mask + (ctx->din->img_sy - 1) * ctx->din->img_ppln * 3,
         ctx->din->img_ppln * 3);

  src = ctx->pimgs_mask;
  dst = ctx->ppruned_mask;
  for (i = 1; i < ctx->din->img_sy - 1; ++i) {
    memcpy(dst, src, 3);
    src += ctx->din->img_ppln * 3;
  }

  src = ctx->pimgs_mask + (ctx->din->img_ppln - 1) * 3;
  dst = ctx->ppruned_mask + (ctx->din->img_ppln - 1) * 3;
  for (i = 1; i < ctx->din->img_sy - 1; ++i) {
    memcpy(dst, src, 3);
    src += ctx->din->img_ppln * 3;
  }

  // meat
  for (row = 1; row < ctx->din->img_sy - 1; row++) {
    dst = ctx->pimgs_mask + (row) * (ctx->din->img_ppln * 3);

    for (col = 1; col < ctx->din->img_sx - 1; col++) {
      dst += 3;
      out = ctx->ppruned_mask + (dst - ctx->pimgs_mask);
      out[0] = out[1] = out[2] = 0;

      // for each image, count color similarities from 8 adjacent pixels
      for (index = 0; index < ctx->din->img_count; ++index) {
        color_count[0] = color_count[1] = color_count[2] = 0;

        for (i = -1; i <= 1; ++i) {
          src = dst + i * (ctx->din->img_ppln * 3);
          for (j = -3; j <= 5; ++j)
            color_count[getChannel(row + i, (col * 3) + j)] +=
                (src[j] >> index) & 1;
        }

        // to force a non-similar pixel to be marked as similar,
        // 8 of 8 neighbors must be marked as similar.
        for (i = 0; i <= 2; ++i)
          if (!(dst[i] & 1 << index)) color_count[i] -= 3;

        for (i = 0; i <= 2; ++i)
          if (color_count[i] >= 5) out[i] |= 1 << index;
      } /* end of image loop */
    } /* end of column loop */
  } /* end of row loop */
}

static void expand_non_similarities(rhdr_context_t* ctx) {
  /*
   * Expand every non-similarity by 1 pixel
   *
   * the mask should already be pruned and validated
   */

  uint8_t* src;
  uint8_t* dst;
  int row, col;
  int rowsz;
  rhdr_in_data_t* din = ctx->din;

  rowsz = din->img_sx * 3;

  /*
   * Row pass: input from ctx->ppruned_mask
   * output to ctx->pimgs_mask + 1 row
   */
  for (col = 0; col < din->img_sx; ++col) {
    src = ctx->ppruned_mask + col * 3;
    dst = ctx->pimgs_mask + col * 3 + rowsz;  // output from second row

    *dst = *src & *(src + rowsz);  // handle first row
    dst[1] = dst[2] = *dst;
    dst += rowsz;
    src += rowsz;

    for (row = 1; row < din->img_sy - 1; ++row) {
      *dst = *(src - rowsz) & *src & *(src + rowsz);  // handle middle rows
      dst[1] = dst[2] = *dst;
      dst += rowsz;
      src += rowsz;
    }

    *dst = *(src - rowsz) & *src;  // handle last row
    dst[1] = dst[2] = *dst;
  }

  /*
   * Column pass: input from ctx->pimgs_mask + 1 row
   * output to ctx->pimgs_mask
   */
  src = ctx->pimgs_mask + rowsz;  // input from second row
  dst = ctx->pimgs_mask;
  for (row = 0; row < din->img_sy; ++row) {
    *dst = *src & *(src + 3);  // handle first column
    dst[1] = dst[2] = *dst;
    dst += 3;
    src += 3;

    for (col = 1; col < din->img_sx - 1; ++col) {
      *dst = *(src - 3) & *src & *(src + 3);  // handle middle columns
      dst[1] = dst[2] = *dst;
      dst += 3;
      src += 3;
    }

    *dst = *(src - 3) & *src;  // handle last column
    dst[1] = dst[2] = *dst;
    dst += 3;
    src += 3;
  }
}

static float neighbors_factor(rhdr_context_t* ctx, int img, int row, int col) {
  /*
  Count the number of neighbouring pixels marked as similar in a 7x7 grid

  With RGB data, because the similarities mask has already been
  passed through 'validate_similarities', all three channels of a pixel
  are marked simultaneously as (non)similar. Thus check only the
  red channel for each pixel of the 7x7 grid.
  */

  int i, j;
  uint8_t* pmask;
  uint8_t* src;
  int count;
  int col_img = col / 3;

  // if this is a border pixel return immediately (not enough adjacent)
  if (row <= 2 || col_img <= 2 || row >= ctx->din->img_sy - 3 ||
      col_img >= ctx->din->img_sx - 3)
    return (float)1;

  pmask = ctx->pimgs_mask + (ctx->din->img_ppln * row * 3) + col_img * 3;

  count = 0;

  // count pixels in a 7x7 grid
  for (i = -3; i <= 3; ++i) {
    src = pmask + (i * ctx->din->img_ppln * 3);
    for (j = -3; j <= 3; ++j) count += (src[j * 3] >> img) & 1;
  }

  if (count < 4 * 7) return 0;
  if (count < 5 * 7) return 0;
  if (count < 6 * 7) return 1;
  if (count < 7 * 7) return 1;
  return 1;
}

static void ave_11_11(rhdr_context_t* ctx, int img, uint16_t* pimg,
                      uint16_t* pref_img, int row, int col, float* out_average,
                      float* out_count_factor) {
  /*
   * This function should be used as following:
   * When there is a pixel that is overexposed/underexposed in the reference
   * image, but the darker/brighter image is not similar at this position, we
   * take an average of the surrounding similar pixels from the darker/brighter
   * image (but only those which correspond to over/underexposed pixels in the
   * reference image) and return an averaged value for this pixel.
   *
   * Also we calculate some weight based on how many similar pixels there are
   * near the given one, so that the "averaged" region will blend gradually
   * with the reference image.
   */

  /*
   * parameters:
   * rhdr_context_t* ctx -- pointer to HDR context
   * int img -- darker/brighter image index
   * uint16_t* pimg -- darker/brighter image pointer
   * uint16_t* pref_img -- reference image pointer
   * int row, int col -- position of the critical pixel
   * float* out_average -- pointer to a float, used to output averaged RGB
   * values float* out_count_factor -- pointer to a float, used to output the
   * weight
   */

  static const uint32_t count_weights[11][11] = {
      /* unity 256 */
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256},
      {256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256}};

  int i, j;
  uint8_t* pmask;
  uint8_t* src;
  int count;
  int count_factor;
  int similar;
  int sum;
  int ref_p;
  int flag;  // signifies whether we should look for underexposed (0) or
             // overexposed pixels (1)

  int i_min, i_max;
  int j_min, j_max;
  i_min = -5;
  i_max = 5;
  j_min = -5;
  j_max = 5;

  pmask = ctx->pimgs_mask + (ctx->din->img_ppln * row * 3) + col;
  flag = pref_img[pmask - ctx->pimgs_mask] > (1 << (RHDR_OUT_BITS_PER_PIX - 1));

  count = 0;
  sum = 0;
  count_factor = 0;

  // count pixels in a 11x11 grid

  if (row <= 4 || col / 3 <= 4 || row >= ctx->din->img_sy - 5 ||
      col / 3 >= ctx->din->img_sx - 5) {
    // if this is a border pixel, modify limits to prevent going outside of
    // buffer
    if (row <= 4) i_min = -row;
    if (row >= ctx->din->img_sy - 5) i_max = ctx->din->img_sy - row - 1;
    if (col / 3 <= 4) j_min = -(col / 3);
    if ((col / 3) >= ctx->din->img_sx - 5)
      j_max = ctx->din->img_sx - (col / 3) - 1;
  }

  for (i = i_min; i <= i_max; ++i) {
    src = pmask + (i * ctx->din->img_ppln * 3);
    for (j = j_min; j <= j_max; ++j) {
      similar = ((src[j * 3] >> img) & 1);  // check the similarity mask
      count_factor += count_weights[i + 5][j + 5] * similar;

      ref_p = pref_img[(src + j * 3) - ctx->pimgs_mask] >>
              (RHDR_OUT_BITS_PER_PIX - 8);

      similar *= ctx->p_weight_tbl[1][ref_p] != 255
                     ? 1
                     : 0;  // check if the pixel is over/underexposed in the
                           // reference image
      similar *=
          (ref_p > 128) ==
          flag;  // check that it is the correct type - over or underexposed

      count += similar;
      sum += pimg[(src + j * 3) - ctx->pimgs_mask] * similar;
    }
  }

  if (count != 0) {
    *out_average = (float)sum / (float)count;
    *out_count_factor =
        (float)count_factor /
        (float)((j_max - j_min + 1) * (i_max - i_min + 1) * 256);
    *out_count_factor += 0.3;
    if (*out_count_factor > 1.0) *out_count_factor = 1.0;
  } else {
    *out_average = 0;
    *out_count_factor = 0;
  }
}

int getChannel(int y, int x) {
  (void)y;
  static const int R = 0;
  static const int G = 1;
  static const int B = 2;
  static const int mas[3] = {R, G, B};
  return mas[x % 3];
}
