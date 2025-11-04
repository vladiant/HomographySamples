#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "proc_img.h"
#include "rhdr.h"
#include "rhdr_prv.h"
#include "stdio.h"

#define ERR_VALUE (-36842122)

#define RHDR_INFO_SUFFIX "_info.txt"
#define RHDR_INDATA_FNAME "rhdr_indata.txt"
#define RHDR_TUNING_FNAME "rhdr_tuning.dat"
#define AWB_OUT_SUFF ".awbout"
#define EXPGAIN_FNAME_EXTENSION ".txt"

// out files for debug
#define OUT_RAW_FNAME_EXTENSION ".raw"
#define OUT_BMP_FNAME_EXTENSION ".bmp"

typedef enum {
  AWB_INFO_TIAWBSIM,  // not maintained
  AWB_INFO_TEST,
  AWB_INFO_TI_OMAP4,  // not maintained
} awb_info_fmt_t;

rhdr_context_t* rhdr_ctx = NULL;
rhdr_in_data_t rhdr_in;

/*
struct sim_in_t{
    uint8_t_t   first_pix_color;//0:R,1:GR,2:GB,3:B //raw_colptn_t
    uint16_t k_awb_r;
    uint16_t k_awb_g;
    uint16_t k_awb_b;
} sim_in;
*/

uint16_t gamma_lin[256] = {
#include "hdr_gamma_lin.txt"
};

uint16_t gamma_hdr_temp[256] = {
#include "hdr_gamma_temp.txt"
};

proc_in_t proc_in = {
    // init values
    256,                                // uint16_t k_awb_r;
    256,                                // uint16_t k_awb_g;
    256,                                // uint16_t k_awb_b;
    gamma_hdr_temp,                     // uint16_t* gamma;
    {0, 0, 0},                          // int16_t pre_offsets[3];  //10bit
    {256, 0, 0, 0, 256, 0, 0, 0, 256},  // int16_t ccm[9];          //S16Q8
    {0, 0, 0},                          // int16_t post_offsets[3]; //10bit
    {77, 150, 29, -43, -85, 128, 128, -107, -21},  // int16_t yuv_m[9]; //S16Q8
    RAW_PTN_RGGB,                                  // raw_colptn_t  col_ptn;
};

rhdr_raw_colptn_t rhdr_colptn_tbl[RAW_PTN_COUNT] = {
    RHDR_RAW_RGGB,  // map RAW_PTN_RGGB
    RHDR_RAW_GRBG,  // map RAW_PTN_GRBG
    RHDR_RAW_GBRG,  // map RAW_PTN_GBRG
    RHDR_RAW_BGGR   // map RAW_PTN_BGGR
};

#define SMEAR_CORE_SIZE (9)
#define SMEAR_SUM_MAX (3000)
#define SMEAR_SUM_STEP (1000)
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
// uint8_t y_weight[256];
#include "pix_val_weight.txt"
    },
    0,                // uint32_t mov_dif_thr_const;
    100,              // 5/256 = 2% //uint32_t mov_dif_thr_mult;
    0,                // SMEAR_CORE_SIZE,//uint8_t  smear_core_size;
    rhdr_smear_core,  // uint16_t*  smear_core;
    sizeof(rhdr_noise_vs_gain) /
        sizeof(rhdr_noise_vs_gain[0]),  // uint8_t rhdr_noise_vs_gain_size
    rhdr_noise_vs_gain,                 // rhdr_noise_vs_gain_t* noise_vs_gain;
    2,                                  // uint8_t dark_snr_thr;

    //    1,//uint32_t percent_min;// 1/1024
    //    1,//uint32_t percent_max;// 1/1024
    //    20,//uint8_t  max_sub;
    //    200,//uint8_t  max_target;
    //    512,//uint16_t max_expand; //U16Q8
};

int read_indata_file(char* file_name, rhdr_in_data_t* rhdr_in,
                     proc_in_t* proc_in);

int read_awb_file(char* file_name, awb_info_fmt_t awb_algo, proc_in_t* proc_in);
void read_exp_gain(uint32_t* img_exp_us, uint16_t* img_gain,
                   char* in_file_name);
int getnum(FILE* input_file);

void blank(void) {  // test
}
void update_out_fname(char* outimg_file_name, char* in_file_name) {
  // creates from input file names in_file_1,in_file_2,in_file_something:
  // output file name as "in_file_1+2+something"

  char* str_o;
  char* str_i;
  if (outimg_file_name[0] == 0) {
    // first image input
    char* point;
    char* slash;
    strcpy(outimg_file_name, in_file_name);
    point = strrchr(outimg_file_name, '.');
    slash = strrchr(outimg_file_name, '\\');
    if (point > slash) {
      *point = 0;  // remove outimg_file_name extension
    }
    return;
  }
  str_o = outimg_file_name;
  str_i = in_file_name;
  while (*str_o && *str_i) {
    if (*str_o != *str_i) {
      break;
    }
    str_o++;
    str_i++;
  }
  // sweep str_o to end
  while (*str_o) {
    str_o++;
  }
  *str_o++ = '+';
  // append the different part of str_i to str_o end
  while (*str_i && (*str_i != '.')) {
    *str_o++ = *str_i++;
  }
  *str_o++ = 0;  // trminating 0
}

int main(int argc, char* argv[]) {
  char indata_file_name[256];
  char tuning_file_name[256];
  char outimg_file_name[256];
  char awb_file_name[256];

  int i;
  int err = 0;
  uint32_t mandatory_filled = 0;
  uint16_t* outrgb16 = NULL;

  // init variables
  outimg_file_name[0] = 0;
  strcpy(indata_file_name, RHDR_INDATA_FNAME);
  strcpy(tuning_file_name, RHDR_TUNING_FNAME);
  rhdr_in.data_pedestal = 0;
  rhdr_in.tune_data = (rhdr_tune_data_t*)&rhdr_tune_data;
  rhdr_in.img_sx = 3264;
  rhdr_in.img_sy = 2448;
  rhdr_in.img_ppln = 3264;
  /* rhdr_in.ref_image = 3; //this is now read in the read_indata_file function
   */

  /*
      rhdr_in.img_exps_us[0] = 1000; //microseconds
      rhdr_in.img_exps_us[1] = 4000; //microseconds
      rhdr_in.img_exps_us[2] = 16000; //microseconds
      rhdr_in.img_exps_us[3] = 1000; //microseconds
      rhdr_in.img_exps_us[4] = 1000; //microseconds
      rhdr_in.img_gains[0] = 256; //multiplier U16Q8
      rhdr_in.img_gains[1] = 256; //multiplier U16Q8
      rhdr_in.img_gains[2] = 256; //multiplier U16Q8
      rhdr_in.img_gains[3] = 256; //multiplier U16Q8
      rhdr_in.img_gains[4] = 256; //multiplier U16Q8
  */
  // read input args
  for (i = 1; i < argc; i++) {
    if (*(argv[i]) == '-') {
      switch (*(++argv[i])) {
        case 'i': {
          char in_file_name[256];

          if (rhdr_in.img_count ==
              sizeof(rhdr_in.ptr_imgs) / sizeof(rhdr_in.ptr_imgs[0])) {
            printf("Up to %d images can be input\n", RHDR_MAX_IMGS);
            printf("Only first %d images input will be used\n", RHDR_MAX_IMGS);
            break;
          }
          mandatory_filled += 1;
          if (mandatory_filled < 11) {
            printf(
                "Please, input a config file by -c before input images by "
                "-i\n");
            exit(1);
          }

          strcpy(in_file_name, argv[++i]);
          read_bmp_16_noshift(&rhdr_in.ptr_imgs[rhdr_in.img_count],
                              rhdr_in.img_sx, rhdr_in.img_sy, in_file_name);

          read_exp_gain(&rhdr_in.img_exps_us[rhdr_in.img_count],
                        &rhdr_in.img_gains[rhdr_in.img_count], in_file_name);

          rhdr_in.img_count++;
          update_out_fname(outimg_file_name, in_file_name);
          if (rhdr_in.img_count == 1) {
            // first image read - copy awb file name
            strcpy(awb_file_name, outimg_file_name);
            strcat(awb_file_name, AWB_OUT_SUFF);
          }
          break;
        }
        case 't': {
          strcpy(tuning_file_name, argv[++i]);
          break;
        }
        case 'c': {
          mandatory_filled += 10;
          strcpy(indata_file_name, argv[++i]);
          if (read_indata_file(indata_file_name, &rhdr_in, &proc_in)) {
            printf("Error reading config file! %s\n", indata_file_name);
            exit(1);
          }
          break;
        }
        default:
          printf("unrecognised command -%s", argv[i]);
      }
    }
  }

  if (mandatory_filled == 11) {
    printf("At least 2 images have to be input\n");
    exit(1);
  }
  if (mandatory_filled < 11) {
    printf("\n");
    printf("Multi Frame High Dynamic Range in YUV space \n");
    printf("\n");
    printf(
        "Usage: %s -c config_file -i input_file -i input_file ... <-t tuning "
        "file> \n\n",
        argv[0]);
    printf("\n");
    printf(
        "-c config_file: if missing, 3264x2448 resolution, 10 bits per pix, "
        "RED fist pixel and 0 data_pedestal are assumed\n");
    printf("\n");
    printf(
        "-i input_file: this statement can be repeated maximum %d times for "
        "inputing up to %d images\n",
        RHDR_MAX_IMGS, RHDR_MAX_IMGS);
    printf("\n");
    printf(
        "-t tuning_file: if missing the values in rhdr_tune_data are used\n");
    printf("\n");
    printf("output file will have name like:\n");
    printf(
        "assume input files written in command line like: img_1,img_n,img_3\n");
    printf("output file img_1+n+3.raw\n");
    printf("\n");
    printf("This simulator also performs simple processing to produce BMP.\n");
    printf(
        "AWB out file is needed for this - it should be named on the first "
        "input image like img_1.awbout.\n");
    printf("\n");
    exit(1);
  }

  if (read_awb_file(awb_file_name, AWB_INFO_TEST, &proc_in)) {
    printf("Error reading config file! %s\n", awb_file_name);
    printf("Proceeding with default values for processing to BMP\n");
  }

  rhdr_in.out_rgb = (uint16_t*)malloc(rhdr_in.img_sx * rhdr_in.img_sy * 3 * 2);
  if (rhdr_in.out_rgb == NULL) {
    printf("Can not allocate memory for image %d bytes\n",
           rhdr_in.img_sx * rhdr_in.img_sy * 3);
    exit(1);
  }

  memcpy(rhdr_in.in_gamma, const_gamma, sizeof(uint16_t) * 256);

  rhdr_in.tune_data = &rhdr_tune_data;
  // rhdr_in.raw_colptn = rhdr_colptn_tbl[proc_in.col_ptn];

  //================== test ==============================
  // write input images as BMPs
  //{
  //    proc_in.gamma = gamma_lin;
  //    int i;
  //    for(i=0; i < rhdr_in.img_count; i++){
  //        char ext[20];
  //        process_raw (&proc_in,
  //                     rhdr_in.img_sx , rhdr_in.img_sy, rhdr_in.img_ppln,
  //                     rhdr_in.ptr_imgs[i],
  //                     &outrgb16,
  //                     rhdr_in.raw_bitsperpix
  //                     );
  //        sprintf(ext,"_%d.bmp",i);
  //        write_bmp(outrgb16, rhdr_in.img_sx, rhdr_in.img_sy,
  //        outimg_file_name, ext);
  //    }
  //}
  //
  //================== test ==============================

  /* needed for RGB HDR */
  rhdr_in.raw_bitsperpix = 16;

  /* meat */
  err |= test_rhdr_create((void**)(&rhdr_ctx));
  err |= test_rhdr_process(rhdr_ctx, &rhdr_in);

  if (err) {
    printf("rhdr ERROR !!!!!!!!\n");
  }

  // err |= test_rhdr_delete(rhdr_ctx);

  //   write_raw(rhdr_in.out_raw, rhdr_in.img_sy, rhdr_in.img_ppln,
  //       strcat(outimg_file_name,OUT_RAW_FNAME_EXTENSION));
  // proc_in.gamma = gamma_hdr_temp;
  //   process_raw (&proc_in,
  //                rhdr_in.img_sx , rhdr_in.img_sy, rhdr_in.img_ppln,
  //                rhdr_in.out_raw,
  //                &outrgb16,
  //                RHDR_OUT_BITS_PER_PIX
  //                );
  {
    // apply gamma
    uint16_t* outimg = (uint16_t*)malloc(rhdr_in.img_sx * rhdr_in.img_sy * 6);
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
      outimg[index] = r1;
      // outimg[index] = rhdr_in.out_rgb[index] << 8;
    }

    write_bmp(outimg, rhdr_in.img_sx, rhdr_in.img_sy, outimg_file_name,
              OUT_BMP_FNAME_EXTENSION);
    free(outimg);
  }
  //==========================test=========================
  // uint16_t hist_bmp[256][256][3];//test
  // write_bmp((uint16_t*)hist_bmp, 256, 256, "hist_bmp", ".bmp");

  //{
  //    proc_in.gamma = gamma_lin;
  //    int i;
  //    for(i=0; i < rhdr_in.img_count; i++){
  //        char ext[20];
  //        process_raw (&proc_in,
  //                     rhdr_in.img_sx , rhdr_in.img_sy, rhdr_in.img_ppln,
  //                     rhdr_in.ptr_imgs[i],
  //                     &outrgb16,
  //                     RHDR_OUT_BITS_PER_PIX//rhdr_in.raw_bitsperpix
  //                     );
  //        sprintf(ext,"_%d.bmp",i);
  //        write_bmp(outrgb16, rhdr_in.img_sx, rhdr_in.img_sy,
  //        outimg_file_name, ext);
  //    }
  //}
  //==========================test=========================
  /* free memory */
  for (i = 0; i < rhdr_in.img_count; i++) {
    if (rhdr_in.ptr_imgs[i]) {
      free(rhdr_in.ptr_imgs[i]);
    }
  }
  if (rhdr_in.out_rgb) {
    free(rhdr_in.out_rgb);
  }
  if (outrgb16) {
    free(outrgb16);
  }
  printf("rhdr simulator\n");

  return 0;
}

int read_indata_file(char* file_name, rhdr_in_data_t* rhdr_in,
                     proc_in_t* proc_in) {
  FILE* f_tuning;
  int32_t dummy;

  if (NULL == (f_tuning = fopen(file_name, "rt"))) {
    printf("Error opening rhdr tuning file %s!\n", file_name);
    return (1);
  }

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->enable = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->raw_bitsperpix = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  proc_in->col_ptn = (raw_colptn_t)dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->data_pedestal = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->img_sx = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->img_sy = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf("Error reading rhdr tuning file!\n");
    return 1;
  }
  rhdr_in->img_ppln = dummy;

  dummy = getnum(f_tuning);
  if (ERR_VALUE == dummy) {
    printf(
        "Note: reference image not specified in config file. Using 3rd image "
        "by default.\n");
    rhdr_in->ref_image = 2;
  } else {
    rhdr_in->ref_image = dummy;
  }

  return 0;
}

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

int read_awb_file(char* file_name, awb_info_fmt_t awb_algo,
                  proc_in_t* proc_in) {
  FILE* f_awb;
  int32_t dummy;
  int i;

  if (NULL == (f_awb = fopen(file_name, "rt"))) {
    printf("Error opening AWB results file %s\n", file_name);
    return (1);
  }

  proc_in->k_awb_r = 256;
  proc_in->k_awb_g = 256;
  proc_in->k_awb_b = 256;

  if (AWB_INFO_TIAWBSIM == awb_algo) {  // TI

    // format in OMAP4 simulator, gnerated by perl
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_r = (uint16_t)dummy;
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_g = (uint16_t)dummy;
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_b = (uint16_t)dummy;
  }

  if (AWB_INFO_TEST == awb_algo) {  // Test format
    dummy = getnum(f_awb);          // Dgain - not used
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }

    dummy = getnum(f_awb);  // Gr gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_g = (uint16_t)dummy;

    dummy = getnum(f_awb);  // R gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_r = (uint16_t)dummy;

    dummy = getnum(f_awb);  // B gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_b = (uint16_t)dummy;

    dummy = getnum(f_awb);  // Gb gain

    // add ccm matrix and offsets
    for (i = 0; i < 3; i++) {
      dummy = getnum(f_awb);
      proc_in->pre_offsets[i] = 0;
    }

    for (i = 0; i < 9; i++) {
      dummy = getnum(f_awb);
      if (ERR_VALUE == dummy) {
        printf("Error reading file!\n");
        return 1;
      }
      proc_in->ccm[i] = dummy;
    }

    dummy = getnum(f_awb);  // CT - not used

    for (i = 0; i < 3; i++) {
      dummy = getnum(f_awb);
      if (ERR_VALUE == dummy) {
        printf("Error reading file!\n");
        return 1;
      }
      proc_in->post_offsets[i] = 0;
    }

    //... rest is not used ( CCM, color temperature)
  }

  if (AWB_INFO_TI_OMAP4 == awb_algo) {  // GBCE out info file

    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    dummy = getnum(f_awb);
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }

    dummy = getnum(f_awb);  // R gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_r = (uint16_t)dummy;

    dummy = getnum(f_awb);  // Gr gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_g = (uint16_t)dummy;

    dummy = getnum(f_awb);  // not used Gb gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }

    dummy = getnum(f_awb);  // B gain
    if (ERR_VALUE == dummy) {
      printf("Error reading file!\n");
      return 1;
    }
    proc_in->k_awb_b = (uint16_t)dummy;
  }

  return 0;
}

void read_exp_gain(uint32_t* img_exp_us, uint16_t* img_gain,
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
    printf("Error opening exp/gain file %s\n", exp_fname);
    exit(1);
  }

  // format in OMAP4 simulator, gnerated by perl
  dummy = getnum(f);
  if (ERR_VALUE == dummy) {
    printf("Error reading file!\n");
    fclose(f);
    exit(1);
  }
  *img_exp_us = dummy;

  dummy = getnum(f);
  if (ERR_VALUE == dummy) {
    printf("Error reading file!\n");
    fclose(f);
    exit(1);
  }
  *img_gain = dummy;

  fclose(f);
}
