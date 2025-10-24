#include "private_include/warpedGridToUniform.h"

#include <stdio.h>

#include "private_include/ImageGridWarpAffine.h"
#include "private_include/warpedGridIndex.h"

void transformToUniformGridLines(
    const GTU_DataT* x, uint32_t xstride, const GTU_DataT* y, uint32_t ystride,
    GTU_DataT* ox, uint32_t oxstride, GTU_DataT* oy, uint32_t oystride,
    uint32_t point_count,

    uint32_t grid_cols, uint32_t grid_rows,

    GTU_DataT ugrid_start_x, GTU_DataT ugrid_start_y,
    GTU_DataT ugrid_cell_width, GTU_DataT ugrid_cell_height,

    GTU_DataT* wgrid_x, uint32_t wgrid_stride_x, GTU_DataT* wgrid_y,
    uint32_t wgrid_stride_y) {
  float H[6];
  int_fast32_t i;
  uint32_t id;
  uint32_t grid_id_x, grid_id_y;
  GTU_DataT cx, cy;
  GTU_DataT x1, x2, x3, x4, y1, y2, y3, y4;
  GTU_DataT X1, X2, X3, X4, Y1, Y2, Y3, Y4;

  for (i = 0; i < point_count; ++i) {
    getWarpedID_linear_stride_indexed(*x, *y, &grid_id_x, &grid_id_y, wgrid_x,
                                      wgrid_stride_x, wgrid_y, wgrid_stride_y,
                                      grid_cols, grid_rows);

    id = grid_id_x + grid_id_y * grid_cols;

    x1 = ugrid_start_x + (grid_id_x + 0) * ugrid_cell_width;
    y1 = ugrid_start_y + (grid_id_y + 0) * ugrid_cell_height;
    x2 = ugrid_start_x + (grid_id_x + 1) * ugrid_cell_width;
    y2 = ugrid_start_y + (grid_id_y + 0) * ugrid_cell_height;
    x3 = ugrid_start_x + (grid_id_x + 0) * ugrid_cell_width;
    y3 = ugrid_start_y + (grid_id_y + 1) * ugrid_cell_height;
    x4 = ugrid_start_x + (grid_id_x + 1) * ugrid_cell_width;
    y4 = ugrid_start_y + (grid_id_y + 1) * ugrid_cell_height;

    X1 = wgrid_x[(id + 0) * wgrid_stride_x];
    Y1 = wgrid_y[(id + 0) * wgrid_stride_y];
    X2 = wgrid_x[(id + 1) * wgrid_stride_x];
    Y2 = wgrid_y[(id + 1) * wgrid_stride_y];
    X3 = wgrid_x[(id + grid_cols + 0) * wgrid_stride_x];
    Y3 = wgrid_y[(id + grid_cols + 0) * wgrid_stride_y];
    X4 = wgrid_x[(id + grid_cols + 1) * wgrid_stride_x];
    Y4 = wgrid_y[(id + grid_cols + 1) * wgrid_stride_y];

    if ((X4 - X1) * (*y - Y1) - (Y4 - Y1) * (*x - X1) > 0)
      findAffine(X1, Y1, X4, Y4, X3, Y3, x1, y1, x4, y4, x3, y3, H);
    else
      findAffine(X1, Y1, X2, Y2, X4, Y4, x1, y1, x2, y2, x4, y4, H);

    cx = *x;
    cy = *y;
    *ox = cx * H[0] + cy * H[1] + H[2];
    *oy = cx * H[3] + cy * H[4] + H[5];

    x += xstride;
    y += ystride;
    ox += oxstride;
    oy += oystride;
  }
}

void transformToUniformGridRandom(
    const GTU_DataT* x, uint32_t xstride, const GTU_DataT* y, uint32_t ystride,
    GTU_DataT* ox, uint32_t oxstride, GTU_DataT* oy, uint32_t oystride,
    uint32_t point_count,

    uint32_t grid_cols, uint32_t grid_rows,

    GTU_DataT ugrid_start_x, GTU_DataT ugrid_start_y,
    GTU_DataT ugrid_cell_width, GTU_DataT ugrid_cell_height,

    GTU_DataT* wgrid_x, uint32_t wgrid_stride_x, GTU_DataT* wgrid_y,
    uint32_t wgrid_stride_y) {
  float H[6];
  int_fast32_t i;
  uint32_t id;
  uint32_t grid_id_x, grid_id_y;
  GTU_DataT cx, cy;
  GTU_DataT x1, x2, x3, x4, y1, y2, y3, y4;
  GTU_DataT X1, X2, X3, X4, Y1, Y2, Y3, Y4;

  for (i = 0; i < point_count; ++i) {
    getWarpedID_random_stride(*x, *y, &grid_id_x, &grid_id_y, wgrid_x,
                              wgrid_stride_x, wgrid_y, wgrid_stride_y,
                              grid_cols, grid_rows);

    id = grid_id_x + grid_id_y * grid_cols;

    x1 = ugrid_start_x + (grid_id_x + 0) * ugrid_cell_width;
    y1 = ugrid_start_y + (grid_id_y + 0) * ugrid_cell_height;
    x2 = ugrid_start_x + (grid_id_x + 1) * ugrid_cell_width;
    y2 = ugrid_start_y + (grid_id_y + 0) * ugrid_cell_height;
    x3 = ugrid_start_x + (grid_id_x + 0) * ugrid_cell_width;
    y3 = ugrid_start_y + (grid_id_y + 1) * ugrid_cell_height;
    x4 = ugrid_start_x + (grid_id_x + 1) * ugrid_cell_width;
    y4 = ugrid_start_y + (grid_id_y + 1) * ugrid_cell_height;

    X1 = wgrid_x[(id + 0) * wgrid_stride_x];
    Y1 = wgrid_y[(id + 0) * wgrid_stride_y];
    X2 = wgrid_x[(id + 1) * wgrid_stride_x];
    Y2 = wgrid_y[(id + 1) * wgrid_stride_y];
    X3 = wgrid_x[(id + grid_cols + 0) * wgrid_stride_x];
    Y3 = wgrid_y[(id + grid_cols + 0) * wgrid_stride_y];
    X4 = wgrid_x[(id + grid_cols + 1) * wgrid_stride_x];
    Y4 = wgrid_y[(id + grid_cols + 1) * wgrid_stride_y];

    if ((X4 - X1) * (*y - Y1) - (Y4 - Y1) * (*x - X1) > 0)
      findAffine(X1, Y1, X4, Y4, X3, Y3, x1, y1, x4, y4, x3, y3, H);
    else
      findAffine(X1, Y1, X2, Y2, X4, Y4, x1, y1, x2, y2, x4, y4, H);

    cx = *x;
    cy = *y;
    *ox = cx * H[0] + cy * H[1] + H[2];
    *oy = cx * H[3] + cy * H[4] + H[5];

    x += xstride;
    y += ystride;
    ox += oxstride;
    oy += oystride;
  }
}
