#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef GTU_DataT
#define GTU_DataT float
#endif  // GTU_DataT

void transformToUniformGridLines(
    const GTU_DataT* x, uint32_t xstride,  // Input points X
    const GTU_DataT* y, uint32_t ystride,  // Input points Y
    GTU_DataT* ox, uint32_t oxstride,      // Output points X
    GTU_DataT* oy, uint32_t oystride,      // Output points Y
    uint32_t point_count,                  // Point count

    uint32_t grid_cols, uint32_t grid_rows,  // The grid cols/rows

    // Uniform Grid Start X/Y and Cell Width/Height
    GTU_DataT ugrid_start_x, GTU_DataT ugrid_start_y,
    GTU_DataT ugrid_cell_width, GTU_DataT ugrid_cell_height,

    // Warped Grid Point Data
    GTU_DataT* wgrid_x, uint32_t wgrid_stride_x, GTU_DataT* wgrid_y,
    uint32_t wgrid_stride_y);

void transformToUniformGridRandom(
    const GTU_DataT* x, uint32_t xstride,  // Input points X
    const GTU_DataT* y, uint32_t ystride,  // Input points Y
    GTU_DataT* ox, uint32_t oxstride,      // Output points X
    GTU_DataT* oy, uint32_t oystride,      // Output points Y
    uint32_t point_count,                  // Point count

    uint32_t grid_cols, uint32_t grid_rows,  // The grid cols/rows

    // Uniform Grid Start X/Y and Cell Width/Height
    GTU_DataT ugrid_start_x, GTU_DataT ugrid_start_y,
    GTU_DataT ugrid_cell_width, GTU_DataT ugrid_cell_height,

    // Warped Grid Point Data
    GTU_DataT* wgrid_x, uint32_t wgrid_stride_x, GTU_DataT* wgrid_y,
    uint32_t wgrid_stride_y);

#ifdef __cplusplus
}
#endif
