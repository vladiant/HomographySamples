/*
 * File:   warpedGridIndex.h
 *
 * Created on April 21, 2012, 5:02 PM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// MSVC inline hack
#ifndef inline
#define inline __inline
#endif

#define WID_DataT float  // Data Type
#define WID_GridT float  // Grid Type

// Vectored Grid
void getWarpedID_linear_vector(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, float* grid, uint8_t grid_ve,
                               uint32_t grid_cols, uint32_t grid_rows);

// Strided Grid with pointer arithmetics
void getWarpedID_linear_stride_pointer(WID_DataT x, WID_DataT y, uint32_t* idx,
                                       uint32_t* idy, float* grid_x,
                                       uint16_t grid_stride_x, float* grid_y,
                                       uint16_t grid_stride_y,
                                       uint32_t grid_cols, uint32_t grid_rows);

// Strided Grid with indexing - recommended function
void getWarpedID_linear_stride_indexed(WID_DataT x, WID_DataT y, uint32_t* idx,
                                       uint32_t* idy, float* grid_x,
                                       uint16_t grid_stride_x, float* grid_y,
                                       uint16_t grid_stride_y,
                                       uint32_t grid_cols, uint32_t grid_rows);

// Strided Grid with indexing and random mesh positions
void getWarpedID_random_stride(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, WID_GridT* grid_x,
                               uint16_t grid_stride_x, WID_GridT* grid_y,
                               uint16_t grid_stride_y, uint32_t grid_cols,
                               uint32_t grid_rows);

// Binary Vector Implementation (slower for small grids ( < 100x100 ))
void getWarpedID_binary_vector(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, WID_GridT* grid, uint8_t grid_ve,
                               uint32_t grid_cols, uint32_t grid_rows);

#ifdef __cplusplus
}
#endif
