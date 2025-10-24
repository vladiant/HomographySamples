#include "private_include/warpedGridIndex.h"

inline char pointInTriangle(WID_DataT x, WID_DataT y, WID_GridT x1,
                            WID_GridT y1, WID_GridT x2, WID_GridT y2,
                            WID_GridT x3, WID_GridT y3) {
  WID_GridT a = (x1 - x) * (y2 - y) - (x2 - x) * (y1 - y);
  WID_GridT b = (x2 - x) * (y3 - y) - (x3 - x) * (y2 - y);
  WID_GridT c = (x3 - x) * (y1 - y) - (x1 - x) * (y3 - y);

  return (a > 0 && b > 0 && c > 0) || (a < 0 && b < 0 && c < 0);
}

char pointInQuad(WID_DataT x, WID_DataT y, WID_GridT x1, WID_GridT y1,
                 WID_GridT x2, WID_GridT y2, WID_GridT x3, WID_GridT y3,
                 WID_GridT x4, WID_GridT y4) {
  WID_GridT a = (x1 - x) * (y2 - y) - (x2 - x) * (y1 - y);
  WID_GridT b = (x2 - x) * (y3 - y) - (x3 - x) * (y2 - y);
  WID_GridT c = (x3 - x) * (y4 - y) - (x4 - x) * (y3 - y);
  WID_GridT d = (x4 - x) * (y1 - y) - (x1 - x) * (y4 - y);

  return (a > 0 && b > 0 && c > 0 && d > 0) ||
         (a < 0 && b < 0 && c < 0 && d < 0);
}

void getWarpedID_random_stride(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, WID_GridT* grid_x,
                               uint16_t grid_stride_x, WID_GridT* grid_y,
                               uint16_t grid_stride_y, uint32_t grid_cols,
                               uint32_t grid_rows) {
  // Find closest grid point
  int_fast32_t px, py;
  // int32_t iy = (grid_rows - 2);
  // int32_t ix = grid_cols - 2;

  int32_t minx = 0, miny = 0;
  int32_t min = 255 * 255 * 255;
  WID_GridT* gx = grid_x;
  WID_GridT* gy = grid_y;

  for (py = 0; py < grid_rows - 1; ++py)
    for (px = 0; px < grid_cols - 1; ++px)
      if (pointInQuad(
              x, y, grid_x[((py + 0) * grid_cols + (px + 0)) * grid_stride_x],
              grid_y[((py + 0) * grid_cols + (px + 0)) * grid_stride_x],
              grid_x[((py + 0) * grid_cols + (px + 1)) * grid_stride_x],
              grid_y[((py + 0) * grid_cols + (px + 1)) * grid_stride_x],
              grid_x[((py + 1) * grid_cols + (px + 1)) * grid_stride_x],
              grid_y[((py + 1) * grid_cols + (px + 1)) * grid_stride_x],
              grid_x[((py + 1) * grid_cols + (px + 0)) * grid_stride_x],
              grid_y[((py + 1) * grid_cols + (px + 0)) * grid_stride_x])) {
        *idx = px;
        *idy = py;
        return;
      }

  // If not found
  for (py = 0; py < grid_rows; ++py) {
    for (px = 0; px < grid_cols; ++px) {
      if ((x - *gx) * (x - *gx) + (y - *gy) * (y - *gy) < min) {
        minx = px;
        miny = py;
        min = (x - *gx) * (x - *gx) + (y - *gy) * (y - *gy);
      }
      gx += grid_stride_x;
      gy += grid_stride_y;
    }
  }

  if (minx >= grid_cols - 1) minx = grid_cols - 2;
  if (miny >= grid_rows - 1) miny = grid_rows - 2;

  *idx = minx;
  *idy = miny;

  //*idx = 0;
  //*idy = 0;
}

void getWarpedID_linear_vector(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, WID_GridT* grid, uint8_t grid_ve,
                               uint32_t grid_cols, uint32_t grid_rows) {
  // Search the vertical - linear
  int_fast32_t i;
  int32_t ix, iy;

  iy = (grid_rows - 2);
  for (i = 1; i < grid_rows - 1; ++i) {
    if ((grid[((i + 1) * grid_cols - 1) * grid_ve + 0] -
         grid[i * grid_cols * grid_ve + 0]) *
                (y - grid[i * grid_cols * grid_ve + 1]) -
            (grid[((i + 1) * grid_cols - 1) * grid_ve + 1] -
             grid[i * grid_cols * grid_ve + 1]) *
                (x - grid[i * grid_cols * grid_ve + 0]) <
        0) {
      iy = i - 1;
      break;
    }
  }

  // Search the horizontal - linear
  ix = grid_cols - 2;
  for (i = 1; i < grid_cols - 1; ++i) {
    if ((grid[((iy + 1) * grid_cols + i) * grid_ve + 0] -
         grid[(iy * grid_cols + i) * grid_ve + 0]) *
                (y - grid[(iy * grid_cols + i) * grid_ve + 1]) -
            (grid[((iy + 1) * grid_cols + i) * grid_ve + 1] -
             grid[(iy * grid_cols + i) * grid_ve + 1]) *
                (x - grid[(iy * grid_cols + i) * grid_ve + 0]) >
        0) {
      ix = i - 1;
      break;
    }
  }

  *idx = ix;
  *idy = iy;
}

void getWarpedID_linear_stride_pointer(WID_DataT x, WID_DataT y, uint32_t* idx,
                                       uint32_t* idy, WID_GridT* grid_x,
                                       uint16_t grid_stride_x,
                                       WID_GridT* grid_y,
                                       uint16_t grid_stride_y,
                                       uint32_t grid_cols, uint32_t grid_rows) {
  WID_GridT *gxl, *gyl, *gxr, *gyr;
  int32_t iy = (grid_rows - 2);
  int32_t ix = grid_cols - 2;
  int_fast32_t i;

  // Search the vertical - linear
  gxl = grid_x + grid_cols * grid_stride_x;
  gyl = grid_y + grid_cols * grid_stride_y;
  gxr = gxl + (grid_cols - 1) * grid_stride_x;
  gyr = gyl + (grid_cols - 1) * grid_stride_y;
  for (i = 1; i < grid_rows - 1; ++i) {
    if ((*gxr - *gxl) * (y - *gyl) - (*gyr - *gyl) * (x - *gxl) < 0) {
      iy = i - 1;
      break;
    }
    gxl += grid_cols * grid_stride_x;
    gyl += grid_cols * grid_stride_y;
    gxr += grid_cols * grid_stride_x;
    gyr += grid_cols * grid_stride_y;
  }

  // Search the horizontal - linear
  gxr = grid_x + iy * grid_cols * grid_stride_x + grid_stride_x;
  gyr = grid_y + iy * grid_cols * grid_stride_y + grid_stride_x;
  gxl = gxr + grid_cols * grid_stride_x;
  gyl = gyr + grid_cols * grid_stride_x;
  for (i = 1; i < grid_cols - 1; ++i) {
    if ((*gxr - *gxl) * (y - *gyl) - (*gyr - *gyl) * (x - *gxl) < 0) {
      ix = i - 1;
      break;
    }
    gxl += grid_stride_x;
    gyl += grid_stride_y;
    gxr += grid_stride_x;
    gyr += grid_stride_y;
  }

  *idx = ix;
  *idy = iy;
}

void getWarpedID_linear_stride_indexed(WID_DataT x, WID_DataT y, uint32_t* idx,
                                       uint32_t* idy, WID_GridT* grid_x,
                                       uint16_t grid_stride_x,
                                       WID_GridT* grid_y,
                                       uint16_t grid_stride_y,
                                       uint32_t grid_cols, uint32_t grid_rows) {
  // Search the vertical - linear
  int_fast32_t i;
  int32_t iy = (grid_rows - 2);
  int32_t ix = grid_cols - 2;

  for (i = 1; i < grid_rows - 1; ++i) {
    if ((grid_x[((i + 1) * grid_cols - 1) * grid_stride_x] -
         grid_x[i * grid_cols * grid_stride_x]) *
                (y - grid_y[i * grid_cols * grid_stride_y]) -
            (grid_y[((i + 1) * grid_cols - 1) * grid_stride_y] -
             grid_y[i * grid_cols * grid_stride_y]) *
                (x - grid_x[i * grid_cols * grid_stride_x]) <
        0) {
      iy = i - 1;
      break;
    }
  }

  // Search the horizontal - linear
  for (i = 1; i < grid_cols - 1; ++i) {
    if ((grid_x[((iy + 1) * grid_cols + i) * grid_stride_x] -
         grid_x[(iy * grid_cols + i) * grid_stride_x]) *
                (y - grid_y[(iy * grid_cols + i) * grid_stride_y]) -
            (grid_y[((iy + 1) * grid_cols + i) * grid_stride_y] -
             grid_y[(iy * grid_cols + i) * grid_stride_y]) *
                (x - grid_x[(iy * grid_cols + i) * grid_stride_x]) >
        0) {
      ix = i - 1;
      break;
    }
  }

  *idx = ix;
  *idy = iy;
}

// *****************************************************************************
void getWarpedID_binary_vector(WID_DataT x, WID_DataT y, uint32_t* idx,
                               uint32_t* idy, WID_GridT* grid, uint8_t grid_ve,
                               uint32_t grid_cols, uint32_t grid_rows) {
  int iy, ix;
  int beg, end;

  beg = 0;
  end = grid_rows - 1;
  while (beg < end - 1) {
    int mid = (beg + end) / 2;
    if ((grid[((mid + 1) * grid_cols - 1) * grid_ve + 0] -
         grid[mid * grid_cols * grid_ve + 0]) *
                (y - grid[mid * grid_cols * grid_ve + 1]) -
            (grid[((mid + 1) * grid_cols - 1) * grid_ve + 1] -
             grid[mid * grid_cols * grid_ve + 1]) *
                (x - grid[mid * grid_cols * grid_ve + 0]) <
        0)
      end = mid;
    else
      beg = mid;
  }
  iy = beg;

  beg = 0;
  end = grid_cols - 1;
  while (beg < end - 1) {
    int mid = (beg + end) / 2;
    if ((grid[((iy + 1) * grid_cols + mid) * grid_ve + 0] -
         grid[(iy * grid_cols + mid) * grid_ve + 0]) *
                (y - grid[(iy * grid_cols + mid) * grid_ve + 1]) -
            (grid[((iy + 1) * grid_cols + mid) * grid_ve + 1] -
             grid[(iy * grid_cols + mid) * grid_ve + 1]) *
                (x - grid[(iy * grid_cols + mid) * grid_ve + 0]) >
        0)
      end = mid;
    else
      beg = mid;
  }
  ix = beg;

  *idx = ix;
  *idy = iy;
}
