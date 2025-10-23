#include <Test/ImageGridWarp/ImageGridWarp.h>
#include <assert.h>
#include <malloc.h>
#include <stdio.h>

#include "private_include/ImageGridWarpAffine.h"
#include "private_include/warpedGridToUniform.h"

void imageGridWarpAffine(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  const int aspect_prc = 16;

  // Uniform grid
  float H[6];
  int_fast32_t cx, cy, x, y;
  int_fast32_t sx, ex, mx, sy, ey;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;

  float dx, dy, ox, oy;

  uint32_t aspect =
      ((src_w * g_r) << aspect_prc) / (g_c * src_h);  // cell aspect
  uint32_t astart;

  cw = (src_w << 16) / g_c;
  ch = (src_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // First Triangle
      findAffine(fsx * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                 g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sx],
                 g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sx], H);

      // We must start from half the aspect
      // so the diagonal can intersect the center of the quad
      astart = (sx << aspect_prc) + (aspect >> 1);
      ox = H[0] * sx + H[2];
      oy = H[3] * sx + H[5];
      for (y = sy; y < ey; ++y) {
        // mx is int, so we convert the q16 astart with rounding
        dx = ox + H[1] * y;
        dy = oy + H[4] * y;
        mx = (astart + (1 << (aspect_prc - 1))) >> aspect_prc;
        for (x = sx; x < mx; ++x) {
          ((float *)dst)[(y * src_w + x) * 2 + 0] = dx;
          ((float *)dst)[(y * src_w + x) * 2 + 1] = dy;
          dx += H[0];
          dy += H[3];
        }
        astart += aspect;  // Next row intersection
      }

      // Second Triangle
      findAffine(fsx * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                 fsx * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                 g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy],
                 g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy],
                 g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy], H);

      astart = (sx << aspect_prc) + (aspect >> 1);
      for (y = sy; y < ey; ++y) {
        mx = (astart + (1 << (aspect_prc - 1))) >> aspect_prc;
        dx = H[0] * mx + H[1] * y + H[2];
        dy = H[3] * mx + H[4] * y + H[5];
        for (x = mx; x < ex; ++x) {
          ((float *)dst)[(y * src_w + x) * 2 + 0] = dx;
          ((float *)dst)[(y * src_w + x) * 2 + 1] = dy;
          dx += H[0];
          dy += H[3];
        }
        astart += aspect;
      }
      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpAffine2(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  const int aspect_prc = 16;

  // Uniform grid
  float H[6];
  int_fast32_t cx, cy, x, y;
  int_fast32_t sx, ex, mx, sy, ey;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;

  // src_w*g_r < 65535
  uint32_t aspect =
      ((src_w * g_r) << aspect_prc) / (g_c * src_h);  // cell aspect
  uint32_t astart;

  cw = (src_w << 16) / g_c;
  ch = (src_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // (height * index + rows/2) / rows
    // rows/2 is needed for rounding i.e. (int)(float + 0.5f)
    // sy = (src_h*(cy + 0) + (g_r>>1))/g_r;
    // ey = (src_h*(cy + 1) + (g_r>>1))/g_r;
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // (width * index + round) / cols
      // sx = (src_w*(cx + 0) + (g_c>>1))/g_c;
      // ex = (src_w*(cx + 1) + (g_c>>1))/g_c;
      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // First Triangle
      findAffine(fsx * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                 // sx, sy,			ex, sy,			ex, ey,
                 g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sx],
                 g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sx], H);

      // We must start from half the aspect
      // so the diagonal can intersect the center of the quad
      astart = (sx << aspect_prc) + (aspect >> 1);
      for (y = sy; y < ey; ++y) {
        // mx is int, so we convert the q16 astart with rounding
        mx = (astart + (1 << (aspect_prc - 1))) >> aspect_prc;
        for (x = sx; x < mx; ++x) {
          ((float *)src)[(y * src_w + x) * 2 + 0] = x;
          ((float *)src)[(y * src_w + x) * 2 + 1] = y;

          ((float *)dst)[(y * src_w + x) * 2 + 0] = x * H[0] + y * H[1] + H[2];
          ((float *)dst)[(y * src_w + x) * 2 + 1] = x * H[3] + y * H[4] + H[5];
        }
        astart += aspect;  // Next row intersection
      }

      // Second Triangle
      findAffine(fsx * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                 fex * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                 // sx, sy,			ex, ey,			ex, sy,
                 g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy],
                 g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy],
                 g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx],
                 g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy], H);

      astart = (sx << aspect_prc) + (aspect >> 1);
      for (y = sy; y < ey; ++y) {
        mx = (astart + (1 << (aspect_prc - 1))) >> aspect_prc;
        for (x = mx; x < ex; ++x) {
          ((float *)src)[(y * src_w + x) * 2 + 0] = x;
          ((float *)src)[(y * src_w + x) * 2 + 1] = y;

          ((float *)dst)[(y * src_w + x) * 2 + 0] = x * H[0] + y * H[1] + H[2];
          ((float *)dst)[(y * src_w + x) * 2 + 1] = x * H[3] + y * H[4] + H[5];
        }
        astart += aspect;
      }
      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearICoord(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Uniform grid
  float H[8];
  int32_t I[8];

  uint_fast32_t cx, cy, x, y;

  int32_t ox, oy;
  int32_t dx, dy;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;

  cw = (src_w << 16) / g_c;
  ch = (src_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // sx, ex - are rounded to the closest int for the bilinear transform,
      // these should be float
      findBilinear(fsx * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                   fex * (1.0f / (1 << 16)), fsy * (1.0f / (1 << 16)),
                   fex * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                   fsx * (1.0f / (1 << 16)), fey * (1.0f / (1 << 16)),
                   g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx],
                   g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy],
                   g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx],
                   g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy],
                   g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx],
                   g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy],
                   g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx],
                   g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy], H);

      // -2 / 2			// Q2.30	e-10
      I[0] = H[0] * (1 << 25);
      I[2] = H[2] * (1 << 25);
      I[4] = H[4] * (1 << 25);
      I[5] = H[5] * (1 << 25);

      // -64 / 64			// Q7.25	e-8
      I[1] = H[1] * (1 << 25);
      I[6] = H[6] * (1 << 25);

      // -65535 / 65535	// Q16.16	e-5
      I[3] = H[3] * (1 << 16);
      I[7] = H[7] * (1 << 16);

      /*
                              printf("*******************\n");
                              printf("%.8f %.8f %.8f %.8f\n%.8f %.8f %.8f
         %.8f\n\n", H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7]);
                              printf("%.8f %.8f %.8f %.8f\n%.8f %.8f %.8f
         %.8f\n\n", I[0]*(1.0f/(1<<31)), I[1]*(1.0f/(1<<25)),
         I[2]*(1.0f/(1<<31)), I[3]*(1.0f/(1<<16)), I[4]*(1.0f/(1<<31)),
         I[5]*(1.0f/(1<<31)), I[6]*(1.0f/(1<<25)), I[7]*(1.0f/(1<<16)));
      */

      for (y = sy; y < ey; ++y) {
        // xya + xb + yc + d == x(ya + b) + (yc + d)
        // xye + xf + yg + h == x(ye + f) + (yg + h)
        ox = ((int64_t)y * I[0] >> 9) + (I[1] >> 9);  // ya + b
        oy = ((int64_t)y * I[4] >> 9) + (I[5] >> 9);  // ye + f
        dx = (int64_t)sx * (int64_t)ox + ((int64_t)y * I[2] >> 9) + I[3];
        dy = (int64_t)sx * (int64_t)oy + ((int64_t)y * I[6] >> 9) + I[7];

        for (x = sx; x < ex; ++x) {
          ((float *)dst)[(y * src_w + x) * 2 + 0] = dx * (1.0f / (1 << 16));
          ((float *)dst)[(y * src_w + x) * 2 + 1] = dy * (1.0f / (1 << 16));
          dx += ox;
          dy += oy;
          //	((float *)dst)[(y*src_w + x)*2 + 0] =
          //((I[0]*(int64_t)x*(int64_t)y>>9) + ((int64_t)x*I[1]>> 9) +
          //((int64_t)y*I[2]>>9) + I[3])/(float)(1<<16);
          //	((float *)dst)[(y*src_w + x)*2 + 1] =
          //((I[4]*(int64_t)x*(int64_t)y>>9) + ((int64_t)x*I[5]>>9) +
          //((int64_t)y*I[6]>> 9) + I[7])/(float)(1<<16);
        }
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearICoordW(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Uniform grid
  int32_t I[8];

  uint_fast32_t cx, cy, x, y;

  int32_t ox, oy;
  int32_t dx, dy;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;

  int32_t bx, by;
  int32_t maxx, minx, maxy, miny;
  int32_t wx[4], wy[4];
  uint8_t side;

  bx = (src_w - 0) << 16;
  by = (src_h - 0) << 16;
  cw = (dst_w << 16) / g_c;
  ch = (dst_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);

      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // sx, ex - are rounded to the closest int for the bilinear transform,
      // these should be float
      findBilinear3I(fsx, fsy, fex, fey, wx[0], wy[0], wx[1], wy[1], wx[2],
                     wy[2], wx[3], wy[3], I);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      // if(miny < 0) side = 2;	else if(maxy > (src_h<<16)) side = 8;
      // if(minx < 0) side --;	else if(maxx > (src_w<<16)) side ++;
      if (miny < (1 << 16) + (1 << 15))
        side = 2;
      else if (maxy > by)
        side = 8;
      if (minx < (1 << 16) + (1 << 15))
        side--;
      else if (maxx > bx)
        side++;

      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy >= (1 << 16) && dx >= (1 << 16)) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy >= (1 << 16)) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy >= (1 << 16) && dx < bx) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);  // (1<<15) the integer part in the src image is
                             // rounded to the closest pixel
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dx >= (1 << 16)) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
              ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
              ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx * (1.0f / (1 << 16));
              ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy * (1.0f / (1 << 16));
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dx < bx) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy < by && dx >= (1 << 16)) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy < by) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            ox = (int64_t)y * I[0] + I[1];  // ya + b
            oy = (int64_t)y * I[4] + I[5];  // ye + f
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3] -
                 (1 << 15);
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7] -
                 (1 << 15);

            for (x = sx; x < ex; ++x) {
              if (dy < by && dx < bx) {
                ((float *)src)[(y * dst_w + x) * 2 + 0] = x;
                ((float *)src)[(y * dst_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] =
                    dx * (1.0f / (1 << 16));
                ((float *)dst)[(y * dst_w + x) * 2 + 1] =
                    dy * (1.0f / (1 << 16));
              }
              dx += ox;
              dy += oy;
            }
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearI(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T *g_x, uint32_t g_sx,  // X, x stride
    IGW_T *g_y, uint32_t g_sy,                              // Y, y stride

    int temp) {
  int x, y;

#ifdef VLA
  float gx[(g_c + 1) * (g_r + 1)];
  float gy[(g_c + 1) * (g_r + 1)];
#else
  float *gx = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  float *gy = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  if (gx == 0) return;
  if (gy == 0) {
    free(gx);
    return;
  }
#endif  // VLA

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      gx[y * (g_c + 1) + x] = (dst_w / (float)g_c) * x;
      gy[y * (g_c + 1) + x] = (dst_h / (float)g_r) * y;
    }
  }

  transformToUniformGridRandom(gx, 1,                  // Input points X
                               gy, 1,                  // Input points Y
                               gx, 1,                  // Output points X
                               gy, 1,                  // Output points Y
                               (g_c + 1) * (g_r + 1),  // Point count

                               g_c + 1, g_r + 1,  // The grid cols/rows

                               // Uniform Grid Start X/Y and Cell Width/Height
                               0, 0, src_w / (float)g_c, src_h / (float)g_r,

                               // Warped Grid Point Data
                               g_x, g_sx, g_y, g_sy);

  imageGridWarpBilinearIBack(
      // Image
      src, src_w, src_h, src_ppl, dst, dst_w, dst_h, dst_ppl,

      // Warped Grid
      g_c, g_r,  // Grid cols/rows
      gx, 1,     // X, x stride
      gy, 1,     // Y, y stride

      temp);

#ifndef VLA
  free(gx);
  free(gy);
#endif  // VLA
}

void imageGridWarpBilinearIBack(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Transformation
  int32_t I[8];

  // Uniform grid
  uint_fast32_t cx, cy, x, y;

  int32_t ox, oy;
  int32_t dx, dy;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;

  int32_t bx, by;
  int32_t maxx, minx, maxy, miny;
  int32_t wx[4], wy[4];
  uint8_t side;

  // Image
  unsigned char *pDstStart;                    // start position
  unsigned char *pDst;                         // current position
  unsigned char *pSrc = (unsigned char *)src;  // current position
  unsigned char *pBil[4];

  // Look up table
  const uint8_t prec = 4;  // 8 bit precision, 1/256, 0.004
                           // 4 bit precision, 1/8, 0.125
                           //!! Don not use prec>7 !!!
                           // The unity of LUT is 2^(2*prec), largest koef is 1
  // So if prec==8, larges coef is 2^16, which exceeds the 16 bit of the LUT
  // Alos, the table size becomes too big with big precision
  const uint8_t preci = 16 - prec;

#ifdef WIN32
  uint16_t LUT[4][1 << 4][1 << 4];
#else
  uint16_t LUT[4][1 << prec][1 << prec];
#endif

  // Calc interpolation table
  // 4 tables are koefs for blending the 4 adjacent pixels:
  //[0]: upper left, [1]:upper right, [2]:lower left, [3]:lower right
  // The interpolated pixel position is presented as index in the table,
  // with 'prec' fractional bits, ranging from 0..1-(1/2^prec)
  // interpolated pixel position 0,0 means upper left corner
  for (y = 0; y < 1 << prec; ++y) {
    for (x = 0; x < 1 << prec; ++x) {
      LUT[0][y][x] = ((1 << prec) - x) * ((1 << prec) - y);  //(1-kx)*(1-ky)
      LUT[1][y][x] = x * ((1 << prec) - y);                  // kx*(1-ky)
      LUT[2][y][x] = ((1 << prec) - x) * y;                  //(1-kx)*ky
      LUT[3][y][x] = x * y;                                  // kx*ky
    }
  }

  src_ppl *= 3;
  dst_ppl *= 3;

  bx = (src_w - 0) << 16;
  by = (src_h - 0) << 16;
  cw = (dst_w << 16) / g_c;
  ch = (dst_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);

      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // sx, ex - are rounded to the closest int for the bilinear transform,
      // these should be float
      findBilinear3I(fsx, fsy, fex, fey, wx[0], wy[0], wx[1], wy[1], wx[2],
                     wy[2], wx[3], wy[3], I);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny < (1 << 16) + (1 << 15))
        side = 2;
      else if (maxy >= by - (1 << 16))
        side = 8;
      if (minx < (1 << 16) + (1 << 15))
        side--;
      else if (maxx >= bx - (1 << 16))
        side++;

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + sx * 3];

      ox = (int64_t)sy * I[0] + I[1];  // ya + b
      oy = (int64_t)sy * I[4] + I[5];  // ye + f

#define PIXEL_INTERP                                                          \
  pBil[0] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 0) * 3;         \
  pBil[1] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 1) * 3;         \
  pBil[2] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 0) * 3;         \
  pBil[3] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 1) * 3;         \
                                                                              \
  pDst[0] = ((*pBil[0]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);                                                       \
                                                                              \
  pDst[1] = ((*pBil[0]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);                                                       \
                                                                              \
  pDst[2] = ((*pBil[0]) *                                                     \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]) *                                                     \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]) *                                                     \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]) *                                                     \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);

      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
              assert((dy >> 16) >= 0 && (dy >> 16) < src_h);

              PIXEL_INTERP;
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 3;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
#undef PIXEL_INTERP
}

void imageGridWarpBilinearIGray(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T *g_x, uint32_t g_sx,  // X, x stride
    IGW_T *g_y, uint32_t g_sy,                              // Y, y stride

    int temp) {
  int x, y;

#ifdef VLA
  float gx[(g_c + 1) * (g_r + 1)];
  float gy[(g_c + 1) * (g_r + 1)];
#else
  float *gx = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  float *gy = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  if (gx == 0) return;
  if (gy == 0) {
    free(gx);
    return;
  }
#endif  // VLA

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      gx[y * (g_c + 1) + x] = (dst_w / (float)g_c) * x;
      gy[y * (g_c + 1) + x] = (dst_h / (float)g_r) * y;
    }
  }

  transformToUniformGridRandom(gx, 1,                  // Input points X
                               gy, 1,                  // Input points Y
                               gx, 1,                  // Output points X
                               gy, 1,                  // Output points Y
                               (g_c + 1) * (g_r + 1),  // Point count

                               g_c + 1, g_r + 1,  // The grid cols/rows

                               // Uniform Grid Start X/Y and Cell Width/Height
                               0, 0, src_w / (float)g_c, src_h / (float)g_r,

                               // Warped Grid Point Data
                               g_x, g_sx, g_y, g_sy);

  imageGridWarpBilinearIBackGray(
      // Image
      src, src_w, src_h, src_ppl, dst, dst_w, dst_h, dst_ppl,

      // Warped Grid
      g_c, g_r,  // Grid cols/rows
      gx, 1,     // X, x stride
      gy, 1,     // Y, y stride

      temp);

#ifndef VLA
  free(gx);
  free(gy);
#endif  // VLA
}

void imageGridWarpBilinearIBackGray(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Transformation
  int32_t I[8];

  // Uniform grid
  uint_fast32_t cx, cy, x, y;

  int32_t ox, oy;
  int32_t dx, dy;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;

  int32_t bx, by;
  int32_t maxx, minx, maxy, miny;
  int32_t wx[4], wy[4];
  uint8_t side;

  // Image
  unsigned char *pDstStart;                    // start position
  unsigned char *pDst;                         // current position
  unsigned char *pSrc = (unsigned char *)src;  // current position
  unsigned char *pBil[4];

  // Look up table
  const uint8_t prec = 4;  // 8 bit precision, 1/256, 0.004
                           // 4 bit precision, 1/8, 0.125
                           //!! Don not use prec>7 !!!
                           // The unity of LUT is 2^(2*prec), largest koef is 1
  // So if prec==8, larges coef is 2^16, which exceeds the 16 bit of the LUT
  // Alos, the table size becomes too big with big precision
  const uint8_t preci = 16 - prec;

#ifdef WIN32
  uint16_t LUT[4][1 << 4][1 << 4];
#else
  uint16_t LUT[4][1 << prec][1 << prec];
#endif

  // Calc interpolation table
  // 4 tables are koefs for blending the 4 adjacent pixels:
  //[0]: upper left, [1]:upper right, [2]:lower left, [3]:lower right
  // The interpolated pixel position is presented as index in the table,
  // with 'prec' fractional bits, ranging from 0..1-(1/2^prec)
  // interpolated pixel position 0,0 means upper left corner
  for (y = 0; y < 1 << prec; ++y) {
    for (x = 0; x < 1 << prec; ++x) {
      LUT[0][y][x] = ((1 << prec) - x) * ((1 << prec) - y);  //(1-kx)*(1-ky)
      LUT[1][y][x] = x * ((1 << prec) - y);                  // kx*(1-ky)
      LUT[2][y][x] = ((1 << prec) - x) * y;                  //(1-kx)*ky
      LUT[3][y][x] = x * y;                                  // kx*ky
    }
  }

  //	src_ppl *= 3;	dst_ppl *= 3;

  bx = (src_w - 0) << 16;
  by = (src_h - 0) << 16;
  cw = (dst_w << 16) / g_c;
  ch = (dst_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);

      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // sx, ex - are rounded to the closest int for the bilinear transform,
      // these should be float
      findBilinear3I(fsx, fsy, fex, fey, wx[0], wy[0], wx[1], wy[1], wx[2],
                     wy[2], wx[3], wy[3], I);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny < (1 << 16) + (1 << 15))
        side = 2;
      else if (maxy >= by - (1 << 16))
        side = 8;
      if (minx < (1 << 16) + (1 << 15))
        side--;
      else if (maxx >= bx - (1 << 16))
        side++;

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + sx];

      ox = (int64_t)sy * I[0] + I[1];  // ya + b
      oy = (int64_t)sy * I[4] + I[5];  // ye + f

#define PIXEL_INTERP                                                          \
  pBil[0] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 0);             \
  pBil[1] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 1);             \
  pBil[2] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 0);             \
  pBil[3] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 1);             \
                                                                              \
  pDst[0] = ((*pBil[0]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);

      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
              assert((dy >> 16) >= 0 && (dy >> 16) < src_h);

              PIXEL_INTERP;
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst++;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
#undef PIXEL_INTERP
}

void imageGridWarpBilinearIGrayAlpha(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T *g_x, uint32_t g_sx,  // X, x stride
    IGW_T *g_y, uint32_t g_sy,                              // Y, y stride

    int temp) {
  int x, y;

#ifdef VLA
  float gx[(g_c + 1) * (g_r + 1)];
  float gy[(g_c + 1) * (g_r + 1)];
#else
  float *gx = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  float *gy = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  if (gx == 0) return;
  if (gy == 0) {
    free(gx);
    return;
  }
#endif  // VLA

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      gx[y * (g_c + 1) + x] = (dst_w / (float)g_c) * x;
      gy[y * (g_c + 1) + x] = (dst_h / (float)g_r) * y;
    }
  }

  transformToUniformGridRandom(gx, 1,                  // Input points X
                               gy, 1,                  // Input points Y
                               gx, 1,                  // Output points X
                               gy, 1,                  // Output points Y
                               (g_c + 1) * (g_r + 1),  // Point count

                               g_c + 1, g_r + 1,  // The grid cols/rows

                               // Uniform Grid Start X/Y and Cell Width/Height
                               0, 0, src_w / (float)g_c, src_h / (float)g_r,

                               // Warped Grid Point Data
                               g_x, g_sx, g_y, g_sy);

  imageGridWarpBilinearIBackGrayAlpha(
      // Image
      src, src_w, src_h, src_ppl, dst, dst_w, dst_h, dst_ppl,

      // Warped Grid
      g_c, g_r,  // Grid cols/rows
      gx, 1,     // X, x stride
      gy, 1,     // Y, y stride

      temp);

#ifndef VLA
  free(gx);
  free(gy);
#endif  // VLA
}

void imageGridWarpBilinearIBackGrayAlpha(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Transformation
  int32_t I[8];

  // Uniform grid
  uint_fast32_t cx, cy, x, y;

  int32_t ox, oy;
  int32_t dx, dy;
  uint32_t cw, ch;
  uint32_t fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;

  int32_t bx, by;
  int32_t maxx, minx, maxy, miny;
  int32_t wx[4], wy[4];
  uint8_t side;

  // Image
  unsigned char *pDstStart;                    // start position
  unsigned char *pDst;                         // current position
  unsigned char *pSrc = (unsigned char *)src;  // current position
  unsigned char *pBil[4];

  // Look up table
  const uint8_t prec = 4;  // 8 bit precision, 1/256, 0.004
                           // 4 bit precision, 1/8, 0.125
                           //!! Don not use prec>7 !!!
                           // The unity of LUT is 2^(2*prec), largest koef is 1
  // So if prec==8, larges coef is 2^16, which exceeds the 16 bit of the LUT
  // Alos, the table size becomes too big with big precision
  const uint8_t preci = 16 - prec;

#ifdef WIN32
  uint16_t LUT[4][1 << 4][1 << 4];
#else
  uint16_t LUT[4][1 << prec][1 << prec];
#endif

  // Calc interpolation table
  // 4 tables are koefs for blending the 4 adjacent pixels:
  //[0]: upper left, [1]:upper right, [2]:lower left, [3]:lower right
  // The interpolated pixel position is presented as index in the table,
  // with 'prec' fractional bits, ranging from 0..1-(1/2^prec)
  // interpolated pixel position 0,0 means upper left corner
  for (y = 0; y < 1 << prec; ++y) {
    for (x = 0; x < 1 << prec; ++x) {
      LUT[0][y][x] = ((1 << prec) - x) * ((1 << prec) - y);  //(1-kx)*(1-ky)
      LUT[1][y][x] = x * ((1 << prec) - y);                  // kx*(1-ky)
      LUT[2][y][x] = ((1 << prec) - x) * y;                  //(1-kx)*ky
      LUT[3][y][x] = x * y;                                  // kx*ky
    }
  }

  src_ppl *= 2;
  dst_ppl *= 2;

  bx = (src_w - 0) << 16;
  by = (src_h - 0) << 16;
  cw = (dst_w << 16) / g_c;
  ch = (dst_h << 16) / g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    // 1<<15 for rounding
    sy = (fsy + (1 << 15)) >> 16;
    ey = (fey + (1 << 15)) >> 16;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx] * (1 << 16);
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx] * (1 << 16);

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy] * (1 << 16);
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy] * (1 << 16);

      sx = (fsx + (1 << 15)) >> 16;
      ex = (fex + (1 << 15)) >> 16;

      // sx, ex - are rounded to the closest int for the bilinear transform,
      // these should be float
      findBilinear3I(fsx, fsy, fex, fey, wx[0], wy[0], wx[1], wy[1], wx[2],
                     wy[2], wx[3], wy[3], I);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny < (1 << 16) + (1 << 15))
        side = 2;
      else if (maxy >= by - (1 << 16))
        side = 8;
      if (minx < (1 << 16) + (1 << 15))
        side--;
      else if (maxx >= bx - (1 << 16))
        side++;

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + 2 * sx];

      ox = (int64_t)sy * I[0] + I[1];  // ya + b
      oy = (int64_t)sy * I[4] + I[5];  // ye + f

#define PIXEL_INTERP                                                          \
  pBil[0] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 0) * 2;         \
  pBil[1] = pSrc + ((dy >> 16) + 0) * src_ppl + ((dx >> 16) + 1) * 2;         \
  pBil[2] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 0) * 2;         \
  pBil[3] = pSrc + ((dy >> 16) + 1) * src_ppl + ((dx >> 16) + 1) * 2;         \
                                                                              \
  pDst[0] = ((*pBil[0]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);                                                       \
                                                                              \
  pDst[1] = ((*pBil[0]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[0][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[1]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[1][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[2]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[2][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (*pBil[3]++) *                                                   \
                 (uint_fast32_t)                                              \
                     LUT[3][(dy & 0xFFFF) >> preci][(dx & 0xFFFF) >> preci] + \
             (1 << (2 * prec - 1))) >>                                        \
            (2 * prec);

      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
              assert((dy >> 16) >= 0 && (dy >> 16) < src_h);

              PIXEL_INTERP;
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx >= 0) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = (int64_t)sx * (int64_t)ox + (int64_t)y * I[2] + I[3];
            dy = (int64_t)sx * (int64_t)oy + (int64_t)y * I[6] + I[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < by - (1 << 16) && dx < bx - (1 << 16)) {
                assert((dx >> 16) >= 0 && (dx >> 16) < src_w);
                assert((dy >> 16) >= 0 && (dy >> 16) < src_h);
                PIXEL_INTERP;
              }
              pDst += 2;
              dx += ox;
              dy += oy;
            }
            pDstStart += dst_ppl;
            ox += I[0];
            oy += I[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
#undef PIXEL_INTERP
}

void imageGridWarpBilinearFBackCoords(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  // Uniform grid
  float H[8];

  uint_fast32_t cx, cy, x, y;

  float ox, oy;
  float dx, dy;
  float cw, ch;
  float fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;
  uint8_t side;
  float maxx, minx, maxy, miny;
  float wx[4], wy[4];

  cw = dst_w / (float)g_c;
  ch = dst_h / (float)g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    sy = fsy + 0.5f;
    ey = fey + 0.5f;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx];
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx];
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx];
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx];

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy];
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy];
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy];
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy];

      // Find the transformation coefficients
      findBilinear(fsx, fsy, fex, fsy, fex, fey, fsx, fey, wx[0], wy[0], wx[1],
                   wy[1], wx[3], wy[3], wx[2], wy[2], H);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny < 0)
        side = 2;
      else if (maxy > src_h)
        side = 8;
      if (minx < 0)
        side--;
      else if (maxx > src_w)
        side++;

      sx = fsx + 0.5f;
      ex = fex + 0.5f;
      // xya + xb + yc + d == x(ya + b) + (yc + d)
      // xye + xf + yg + h == x(ye + f) + (yg + h)
      ox = sy * H[0] + H[1];  // ya + b
      oy = sy * H[4] + H[5];  // ye + f

      // For each side a different check is needed
      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dx >= 0 && dy >= 0) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dy >= 0) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx < src_w) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dx >= 0) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              ((float *)src)[(y * src_w + x) * 2 + 0] = x;
              ((float *)src)[(y * src_w + x) * 2 + 1] = y;
              ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
              ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dx < src_w) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dx >= 0 && dy < src_h) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dy < src_h) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            for (x = sx; x < ex; ++x) {
              if (dy < src_h && dx < src_w) {
                ((float *)src)[(y * src_w + x) * 2 + 0] = x;
                ((float *)src)[(y * src_w + x) * 2 + 1] = y;
                ((float *)dst)[(y * dst_w + x) * 2 + 0] = dx;
                ((float *)dst)[(y * dst_w + x) * 2 + 1] = dy;
              }
              dx += ox;
              dy += oy;
            }
            ox += H[0];
            oy += H[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearFNearest(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  float H[8];  // Transformation

  // Loop Pixel Iterations
  uint_fast32_t cx, cy, x, y;

  float ox, oy;
  float dx, dy;
  float cw, ch;
  float fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;
  uint8_t side;
  float maxx, minx, maxy, miny;
  float wx[4], wy[4];

  // Image
  unsigned char *pDstStart;                    // start position
  unsigned char *pDst;                         // current position
  unsigned char *pSrc = (unsigned char *)src;  // current position

  src_ppl *= 3;
  dst_ppl *= 3;

  // End of Declarations

  cw = dst_w / (float)g_c;
  ch = dst_h / (float)g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    sy = fsy + 0.5f;
    ey = fey + 0.5f;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx];
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx];
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx];
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx];

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy];
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy];
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy];
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy];

      // Find the transformation coefficients
      findBilinear(fsx, fsy, fex, fsy, fex, fey, fsx, fey, wx[0], wy[0], wx[1],
                   wy[1], wx[3], wy[3], wx[2], wy[2], H);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny < 0)
        side = 2;
      else if (maxy > src_h)
        side = 8;
      if (minx < 0)
        side--;
      else if (maxx > src_w)
        side++;

      sx = fsx + 0.5f;
      ex = fex + 0.5f;
      // xya + xb + yc + d == x(ya + b) + (yc + d)
      // xye + xf + yg + h == x(ye + f) + (yg + h)
      ox = sy * H[0] + H[1];  // ya + b
      oy = sy * H[4] + H[5];  // ye + f

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + sx * 3];

      // For each side a different check is needed
      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0 && dy >= 0) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy >= 0 && dx < src_w) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              // *(uint32_t*)pDst = *(uint32_t*)&pSrc[((int)dy*src_ppl +
              // (int)dx*3)];

              pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
              pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
              pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];

              /*
                                                              int c =
                 *(uint32_t*)&pSrc[((int)dy*src_ppl + (int)dx*3)]; pDst[0] =
                 (c>>0)&0xFF; pDst[1] = (c>>8)&0xFF; pDst[2] = (c>>16)&0xFF;
              */

              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx < src_w) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dx >= 0 && dy < src_h) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < src_h) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              if (dy < src_h && dx < src_w) {
                pDst[0] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 0];
                pDst[1] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 1];
                pDst[2] = pSrc[((int)dy * src_ppl + (int)dx * 3) + 2];
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearF(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T *g_x, uint32_t g_sx,  // X, x stride
    IGW_T *g_y, uint32_t g_sy,                              // Y, y stride

    int temp) {
  int x, y;
#ifdef WIN32
  float *gx = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  float *gy = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  if (gx == 0) return;
  if (gy == 0) {
    free(gx);
    return;
  }
#else
  float gx[(g_c + 1) * (g_r + 1)];
  float gy[(g_c + 1) * (g_r + 1)];
#endif

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      gx[y * (g_c + 1) + x] = (dst_w / (float)g_c) * x;
      gy[y * (g_c + 1) + x] = (dst_h / (float)g_r) * y;
    }
  }

  transformToUniformGridRandom(gx, 1,                  // Input points X
                               gy, 1,                  // Input points Y
                               gx, 1,                  // Output points X
                               gy, 1,                  // Output points Y
                               (g_c + 1) * (g_r + 1),  // Point count

                               g_c + 1, g_r + 1,  // The grid cols/rows

                               // Uniform Grid Start X/Y and Cell Width/Height
                               0, 0, src_w / (float)g_c, src_h / (float)g_r,

                               // Warped Grid Point Data
                               g_x, g_sx, g_y, g_sy);

  // imageGridWarpBilinearFNearest(
  imageGridWarpBilinearFBack(
      // Image
      src, src_w, src_h, src_ppl, dst, dst_w, dst_h, dst_ppl,

      // Warped Grid
      g_c, g_r,  // Grid cols/rows
      gx, 1,     // X, x stride
      gy, 1,     // Y, y stride

      temp);

#ifdef WIN32
  free(gx);
  free(gy);
#endif
}

void imageGridWarpBilinearFCoords(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T *g_x, uint32_t g_sx,  // X, x stride
    IGW_T *g_y, uint32_t g_sy,                              // Y, y stride

    int temp) {
  int x, y;

#ifdef WIN32
  float *gx = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  float *gy = (float *)malloc((g_c + 1) * (g_r + 1) * sizeof(float));
  if (gx == 0) return;
  if (gy == 0) {
    free(gx);
    return;
  }
#else
  float gx[(g_c + 1) * (g_r + 1)];
  float gy[(g_c + 1) * (g_r + 1)];
#endif

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      gx[y * (g_c + 1) + x] = x * dst_w / (float)g_c;
      gy[y * (g_c + 1) + x] = y * dst_h / (float)g_r;
    }
  }

  transformToUniformGridRandom(gx, 1,                  // Input points X
                               gy, 1,                  // Input points Y
                               gx, 1,                  // Output points X
                               gy, 1,                  // Output points Y
                               (g_c + 1) * (g_r + 1),  // Point count

                               g_c + 1, g_r + 1,  // The grid cols/rows

                               // Uniform Grid Start X/Y and Cell Width/Height
                               0, 0, src_w / (float)g_c, src_h / (float)g_r,

                               // Warped Grid Point Data
                               g_x, g_sx, g_y, g_sy);

  for (y = 0; y <= g_r; ++y) {
    for (x = 0; x <= g_c; ++x) {
      *g_x = gx[y * (g_c + 1) + x];
      *g_y = gy[y * (g_c + 1) + x];
      g_x += g_sx;
      g_y += g_sy;
    }
  }

#ifdef WIN32
  free(gx);
  free(gy);
#endif
}

void imageGridWarpBilinearFBack(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  float H[8];  // Transformation

  // Loop Pixel Iterations
  uint_fast32_t cx, cy, x, y;

  float ox, oy;
  float dx, dy;
  float cw, ch;
  float fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;
  uint8_t side;
  float maxx, minx, maxy, miny;
  float wx[4], wy[4];

  int dstx, dsty;
  float ddx, ddy;

  // Image
  unsigned char *pDstStart;                    // start position
  unsigned char *pDst;                         // current position
  unsigned char *pSrc = (unsigned char *)src;  // current position

  src_ppl *= 3;
  dst_ppl *= 3;

  // End of Declarations

  cw = dst_w / (float)g_c;
  ch = dst_h / (float)g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    sy = fsy + 0.5f;
    ey = fey + 0.5f;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx];
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx];
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx];
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx];

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy];
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy];
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy];
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy];

      // Find the transformation coefficients
      findBilinear(fsx, fsy, fex, fsy, fex, fey, fsx, fey, wx[0], wy[0], wx[1],
                   wy[1], wx[3], wy[3], wx[2], wy[2], H);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny <= 1)
        side = 2;
      else if (maxy >= src_h - 1)
        side = 8;
      if (minx <= 1)
        side--;
      else if (maxx >= src_w - 1)
        side++;

      sx = fsx + 0.5f;
      ex = fex + 0.5f;
      // xya + xb + yc + d == x(ya + b) + (yc + d)
      // xye + xf + yg + h == x(ye + f) + (yg + h)
      ox = sy * H[0] + H[1];  // ya + b
      oy = sy * H[4] + H[5];  // ye + f

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + sx * 3];

      // For each side a different check is needed
      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dstx >= 0 && dsty >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dsty >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dsty >= 0 && dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dstx >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                            (1 - ddx) * (1 - ddy) +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                            (0 + ddx) * (1 - ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                            (1 - ddx) * (0 + ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                            (0 + ddx) * (0 + ddy);
              pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                            (1 - ddx) * (1 - ddy) +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                            (0 + ddx) * (1 - ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                            (1 - ddx) * (0 + ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                            (0 + ddx) * (0 + ddy);
              pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                            (1 - ddx) * (1 - ddy) +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                            (0 + ddx) * (1 - ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                            (1 - ddx) * (0 + ddy) +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                            (0 + ddx) * (0 + ddy);

              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dstx >= 0 && dsty < src_h) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;

              if (dsty < src_h - 1) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx);
              ddx = dx - dstx;
              dsty = (int)(dy);
              ddy = dy - dsty;
              if (dsty < src_h && dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (0 + ddx) * (0 + ddy);
                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (0 + ddx) * (0 + ddy);
                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (1 - ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (0 + ddx) * (0 + ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}

void imageGridWarpBilinearFLUT(
    // Image
    void *src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void *dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T *g_x, uint32_t g_sx,   // X, x stride
    IGW_T *g_y, uint32_t g_sy,   // Y, y stride

    int temp) {
  float H[8];  // Transformation

  // Loop Pixel Iterations
  uint_fast32_t cx, cy, x, y;

  float ox, oy;
  float dx, dy;
  float cw, ch;
  float fsx = 0, fex = 0, fsy = 0, fey = 0;
  uint_fast32_t sx, ex, sy, ey;
  uint8_t side;
  float maxx, minx, maxy, miny;
  float wx[4], wy[4];

  int dstx, dsty;
  float ddx, ddy;

  // Image
  unsigned char *pDstStart;  // start position
  unsigned char *pDst;       // current position
  unsigned char *pSrc;       // current position

  float LUT[16 * 16][4];
  for (y = 0; y < 16; ++y)
    for (x = 0; x < 16; ++x) {
      LUT[(y << 4) + x][0] = (x / 15.0f) * (y / 15.0f);
      LUT[(y << 4) + x][1] = (15.0f - x / 15.0f) * (y / 15.0f);
      LUT[(y << 4) + x][2] = (x / 15.0f) * (15.0f - y / 15.0f);
      LUT[(y << 4) + x][3] = (15.0f - x / 15.0f) * (15.0f - y / 15.0f);
    }

  // Image
  pSrc = (unsigned char *)src;  // current position

  src_ppl *= 3;
  dst_ppl *= 3;

  // End of Declarations

  cw = dst_w / (float)g_c;
  ch = dst_h / (float)g_r;
  fey = ch;

  // For each cell in the uniform grid
  for (cy = 0; cy < g_r; ++cy) {
    sy = fsy + 0.5f;
    ey = fey + 0.5f;

    fsx = 0;
    fex = cw;
    for (cx = 0; cx < g_c; ++cx) {
      // Warped Grid Points
      wx[0] = g_x[((cy + 0) * (g_c + 1) + cx + 0) * g_sx];
      wx[1] = g_x[((cy + 0) * (g_c + 1) + cx + 1) * g_sx];
      wx[2] = g_x[((cy + 1) * (g_c + 1) + cx + 0) * g_sx];
      wx[3] = g_x[((cy + 1) * (g_c + 1) + cx + 1) * g_sx];

      wy[0] = g_y[((cy + 0) * (g_c + 1) + cx + 0) * g_sy];
      wy[1] = g_y[((cy + 0) * (g_c + 1) + cx + 1) * g_sy];
      wy[2] = g_y[((cy + 1) * (g_c + 1) + cx + 0) * g_sy];
      wy[3] = g_y[((cy + 1) * (g_c + 1) + cx + 1) * g_sy];

      // Find the transformation coefficients
      findBilinear(fsx, fsy, fex, fsy, fex, fey, fsx, fey, wx[0], wy[0], wx[1],
                   wy[1], wx[3], wy[3], wx[2], wy[2], H);

      // Find the cell AABB
      minx = maxx = wx[0];
      miny = maxy = wy[0];
      if (wx[1] < minx) minx = wx[1];
      if (wx[1] > maxx) maxx = wx[1];
      if (wx[2] < minx) minx = wx[2];
      if (wx[2] > maxx) maxx = wx[2];
      if (wx[3] < minx) minx = wx[3];
      if (wx[3] > maxx) maxx = wx[3];

      if (wy[1] < miny) miny = wy[1];
      if (wy[1] > maxy) maxy = wy[1];
      if (wy[2] < miny) miny = wy[2];
      if (wy[2] > maxy) maxy = wy[2];
      if (wy[3] < miny) miny = wy[3];
      if (wy[3] > maxy) maxy = wy[3];

      // Find which side of the cell intersects the src image borders (numpad)
      side = 5;
      if (miny <= 1)
        side = 2;
      else if (maxy >= src_h - 1)
        side = 8;
      if (minx <= 1)
        side--;
      else if (maxx >= src_w - 1)
        side++;

      sx = fsx + 0.5f;
      ex = fex + 0.5f;
      // xya + xb + yc + d == x(ya + b) + (yc + d)
      // xye + xf + yg + h == x(ye + f) + (yg + h)
      ox = sy * H[0] + H[1];  // ya + b
      oy = sy * H[4] + H[5];  // ye + f

      pDstStart = &((unsigned char *)dst)[sy * dst_ppl + sx * 3];

      // For each side a different check is needed
      switch (side) {
        case 1:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dstx >= 0 && dsty >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 2:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dsty >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 3:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dsty >= 0 && dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 4:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dstx >= 0) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 5:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              /*
                                                              int dstx =
                 (int)(dx + 0.5f) - 1; int dsty = (int)(dy + 0.5f) - 1; float
                 ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f)); float ddy = 1.0f
                 - ((dy + 0.5f) - (int)(dy + 0.5f));
              */
              int dstx = (int)(dx + 0.5f) - 1;
              int dsty = (int)(dy + 0.5f) - 1;
              float ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              float ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][0] +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][1] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][2] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][3];

              pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][0] +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][1] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][2] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][3];

              pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][0] +
                        pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][1] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][2] +
                        pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                            LUT[(((int)ddy * 16) << 4) + (int)ddx * 16][3];

              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 6:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 7:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dstx >= 0 && dsty < src_h) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 8:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dsty < src_h) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
        case 9:
          for (y = sy; y < ey; ++y) {
            dx = sx * ox + y * H[2] + H[3];
            dy = sx * oy + y * H[6] + H[7];

            pDst = pDstStart;
            for (x = sx; x < ex; ++x) {
              dstx = (int)(dx + 0.5f) - 1;
              dsty = (int)(dy + 0.5f) - 1;
              ddx = 1.0f - ((dx + 0.5f) - (int)(dx + 0.5f));
              ddy = 1.0f - ((dy + 0.5f) - (int)(dy + 0.5f));

              if (dsty < src_h && dstx < src_w) {
                pDst[0] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 0] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 0] *
                              (1 - ddx) * (1 - ddy);

                pDst[1] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 1] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 1] *
                              (1 - ddx) * (1 - ddy);

                pDst[2] = pSrc[((dsty + 0) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (0 + ddy) +
                          pSrc[((dsty + 0) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (0 + ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 0) * 3) + 2] *
                              (0 + ddx) * (1 - ddy) +
                          pSrc[((dsty + 1) * src_ppl + (dstx + 1) * 3) + 2] *
                              (1 - ddx) * (1 - ddy);
              }
              dx += ox;
              dy += oy;
              pDst += 3;
            }
            pDstStart += dst_ppl;
            ox += H[0];
            oy += H[4];
          }
          break;
      }

      fsx = fex;
      fex += cw;
    }
    fsy = fey;
    fey += ch;
  }
}
