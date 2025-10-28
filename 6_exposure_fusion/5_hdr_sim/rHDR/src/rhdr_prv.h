#pragma once

//-----------------------------------------------------------------------------
//-------------------- DEFINES
//---------------------------------------------------
//-----------------------------------------------------------------------------
#define RHDR_PIXEL_FTR_MASK 0xF000
#define RHDR_PIXEL_FTR_MOVING 0x8000
#define RHDR_SAT_LEVEL ((1 << RHDR_OUT_BITS_PER_PIX) - 20)
#define RHDR_SAT_LEVEL_DIF_DIFF ((1 << RHDR_OUT_BITS_PER_PIX) - 256)
#define RHDR_WEIGHT_TABLE_COUNT (3)
//-----------------------------------------------------------------------------
//-------------------- INCLUDES
//---------------------------------------------------
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//-------------------- DEFS ---------------------------------------------------
//-----------------------------------------------------------------------------
typedef struct {
  uint32_t rhdr_version;
  uint32_t unknown_overexp;
  uint32_t unknown_dark;
} rhdr_info_t;

// typedef struct {
// } rhdr_tune_data_copy_t;

/* the calc memory buffer for rhdr is once allocated */
typedef struct {
  rhdr_info_t info;  // keep that first in ctx to easy debugging
  //    uint16_t* pimgs[RHDR_MAX_IMGS];
  //    uint16_t* pmin_imgs[RHDR_MAX_IMGS];
  //    uint16_t* pmax_imgs[RHDR_MAX_IMGS];
  uint8_t p_weight_tbl[RHDR_WEIGHT_TABLE_COUNT][256];
  uint32_t exp_ratio[RHDR_MAX_IMGS];
  rhdr_in_data_t* din;
  uint32_t col_ptn;
  uint8_t* pimgs_mask;

  uint8_t* ppruned_mask;

} rhdr_context_t;
