/*
 * File:   klt.cpp
 *
 * Created on June 4, 2012, 2:17 PM
 */

#include "klt.h"

#include "sad_86.h"

void cone_fit(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
              int width, int height, float* coeffs) {
  uint32_t z[9];
  z[0] = sad_fst(kernel, img - width - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[1] = sad_fst(kernel, img - width + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[2] = sad_fst(kernel, img - width + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[3] = sad_fst(kernel, img - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[4] = sad_fst(kernel, img + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[5] = sad_fst(kernel, img + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[6] = sad_fst(kernel, img + width - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[7] = sad_fst(kernel, img + width + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[8] = sad_fst(kernel, img + width + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);

  // printf("%d\t\t%d\t\t%d\n%d\t\t%d\t\t%d\n%d\t\t%d\t\t%d\n\n",
  //		z[0], z[1], z[2], z[3], z[4], z[5], z[6], z[7], z[8]);

  z[0] *= z[0];
  z[1] *= z[1];
  z[2] *= z[2];
  z[3] *= z[3];
  z[4] *= z[4];
  z[5] *= z[5];
  z[6] *= z[6];
  z[7] *= z[7];
  z[8] *= z[8];

  float d = (int32_t)(-z[0] - z[3] - z[6] + z[2] + z[5] + z[8]) / 6.0f;
  float e = (int32_t)(-z[0] - z[1] - z[2] + z[6] + z[7] + z[8]) / 6.0f;
  float c = (int32_t)(z[0] - z[2] - z[6] + z[8]) / 4.0f;

  float a =
      (int32_t)(z[0] + z[2] + z[3] + z[5] + z[6] + z[8]) / 2.0f -
      (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] + z[7] + z[8]) /
          3.0f;
  float b =
      (int32_t)(z[0] + z[1] + z[2] + z[6] + z[7] + z[8]) / 2.0f -
      (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] + z[7] + z[8]) /
          3.0f;
  float f = -(int32_t)(z[0] + z[2] + z[3] + z[5] + z[6] + z[8]) / 3.0f -
            (int32_t)(z[0] + z[1] + z[2] + z[6] + z[7] + z[8]) / 3.0f +
            5.0f *
                (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] +
                          z[7] + z[8]) /
                9.0f;

  coeffs[0] = a;
  coeffs[1] = b;
  coeffs[2] = c;
  coeffs[3] = d;
  coeffs[4] = e;
  coeffs[5] = f;

  coeffs[6] = (-2 * coeffs[1] * coeffs[3] + 1 * coeffs[2] * coeffs[4]) /
              (4 * coeffs[0] * coeffs[1] - coeffs[2] * coeffs[2]);
  coeffs[7] = (+1 * coeffs[3] * coeffs[2] - 2 * coeffs[0] * coeffs[4]) /
              (4 * coeffs[0] * coeffs[1] - coeffs[2] * coeffs[2]);

  if (coeffs[6] > 1.0f || coeffs[6] < -1.0f || coeffs[7] > 1.0f ||
      coeffs[7] < -1.0f) {
    coeffs[6] = 0.0f;
    coeffs[7] = 0.0f;
  }
}

void paraboloid_fit(uint8_t* kernel, int kernelWidth, int kernelHeight,
                    uint8_t* img, int width, int height, float* coeffs) {
  uint32_t z[9];
  z[0] = sad_fst(kernel, img - width - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[1] = sad_fst(kernel, img - width + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[2] = sad_fst(kernel, img - width + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[3] = sad_fst(kernel, img - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[4] = sad_fst(kernel, img + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[5] = sad_fst(kernel, img + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[6] = sad_fst(kernel, img + width - 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[7] = sad_fst(kernel, img + width + 0, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);
  z[8] = sad_fst(kernel, img + width + 1, kernelWidth, kernelHeight,
                 width - kernelWidth, width - kernelWidth);

  // printf("%d\t\t%d\t\t%d\n%d\t\t%d\t\t%d\n%d\t\t%d\t\t%d\n\n",
  //		z[0], z[1], z[2], z[3], z[4], z[5], z[6], z[7], z[8]);

  float d = (int32_t)(-z[0] - z[3] - z[6] + z[2] + z[5] + z[8]) / 6.0f;
  float e = (int32_t)(-z[0] - z[1] - z[2] + z[6] + z[7] + z[8]) / 6.0f;
  float c = (int32_t)(z[0] - z[2] - z[6] + z[8]) / 4.0f;

  float a =
      (int32_t)(z[0] + z[2] + z[3] + z[5] + z[6] + z[8]) / 2.0f -
      (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] + z[7] + z[8]) /
          3.0f;
  float b =
      (int32_t)(z[0] + z[1] + z[2] + z[6] + z[7] + z[8]) / 2.0f -
      (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] + z[7] + z[8]) /
          3.0f;
  float f = -(int32_t)(z[0] + z[2] + z[3] + z[5] + z[6] + z[8]) / 3.0f -
            (int32_t)(z[0] + z[1] + z[2] + z[6] + z[7] + z[8]) / 3.0f +
            5.0f *
                (int32_t)(z[0] + z[1] + z[2] + z[3] + z[4] + z[5] + z[6] +
                          z[7] + z[8]) /
                9.0f;

  coeffs[0] = a;
  coeffs[1] = b;
  coeffs[2] = c;
  coeffs[3] = d;
  coeffs[4] = e;
  coeffs[5] = f;

  coeffs[6] = (-2 * coeffs[1] * coeffs[3] + 1 * coeffs[2] * coeffs[4]) /
              (4 * coeffs[0] * coeffs[1] - coeffs[2] * coeffs[2]);
  coeffs[7] = (+1 * coeffs[3] * coeffs[2] - 2 * coeffs[0] * coeffs[4]) /
              (4 * coeffs[0] * coeffs[1] - coeffs[2] * coeffs[2]);

  if (coeffs[6] > 1.0f || coeffs[6] < -1.0f || coeffs[7] > 1.0f ||
      coeffs[7] < -1.0f) {
    coeffs[6] = 0.0f;
    coeffs[7] = 0.0f;
  }
}

void sad_brute(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
               int width, int height, char* out, int* minx, int* miny) {
  uint32_t sum;
  uint32_t minsum = (~0);
  int mx, my;
  for (int y = 0; y < height - kernelHeight; ++y) {
    for (int x = 0; x < width - kernelWidth; ++x) {
      sum = sad_fst(kernel, img, kernelWidth, kernelHeight, width - kernelWidth,
                    width - kernelWidth);
      if (sum < minsum) {
        minsum = sum;
        mx = x;
        my = y;
      }
      if (out != 0) *out++ = sum;
      img++;
    }
    if (out != 0) out += kernelWidth;
    img += kernelWidth;
  }
  if (minx != 0) *minx = mx;
  if (miny != 0) *miny = my;
}

#include <stdio.h>
// template <typename outT>
void sad_brute(uint8_t* kernel, int kernelWidth, int kernelHeight, int sx,
               int sy, int swid, int shei, uint8_t* img, int width, int height,
               char* out, int* minx, int* miny) {
  uint32_t sum;
  uint32_t minsum = (~0);
  int mx = 0, my = 0;
  // IMG pointer arithmetics
  swid += sx;
  shei += sy;
  for (int y = sy; y < shei; ++y) {
    for (int x = sx; x < swid; ++x) {
      sum = sad_fst(kernel, img + y * width + x, kernelWidth, kernelHeight,
                    width - kernelWidth, width - kernelWidth);
      if (sum < minsum) {
        minsum = sum;
        mx = x;
        my = y;
      }
      if (out != 0) *out++ = sum;
    }
    if (out != 0) out += kernelWidth;
  }
  if (minx != 0) *minx = mx;
  if (miny != 0) *miny = my;
}

#include <cstdlib>
#include <iostream>
using namespace std;
void klt_deriv(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
               int width, int height, float* dx, float* dy) {
  int32_t ssidx = 0;
  int32_t ssidy = 0;
  int32_t ssix = 0;
  int32_t ssiy = 0;
  int32_t I0dx = 0;
  int32_t I0dy = 0;

  for (int y = 0; y < kernelHeight; ++y) {
    for (int x = 0; x < kernelWidth; ++x) {
      I0dx = (kernel[1] - kernel[-1]) / 2;
      I0dy = (kernel[width] - kernel[-width]) / 2;

      ssix += I0dx * (kernel[0] - img[0]);
      ssiy += I0dy * (kernel[0] - img[0]);

      ssidx += I0dx * I0dx;
      ssidy += I0dy * I0dy;

      kernel++;
      img++;
    }
    kernel += width - kernelWidth;
    img += width - kernelWidth;
  }

  *dx = ssix / (float)ssidx;
  *dy = ssiy / (float)ssidy;
}

void blockCorrelationCurves(uint8_t* krn, int kernelWidth, int kernelHeight,
                            uint8_t* img, int width, int height, int32_t* crlX,
                            int32_t* crlY) {
  int32_t sumA[2][(kernelWidth > kernelHeight) ? kernelWidth : kernelHeight];
  int32_t sumB[2][(kernelWidth > kernelHeight) ? kernelWidth : kernelHeight];

  for (int x = 0; x < kernelWidth; ++x) {
    sumA[0][x] = 0;
    sumB[0][x] = 0;
    for (int y = 0; y < kernelHeight; ++y) {
      sumA[0][x] += krn[width * y + x];
      sumB[0][x] += img[width * y + x];
    }
  }

  for (int y = 0; y < kernelHeight; ++y) {
    sumA[1][y] = 0;
    sumB[1][y] = 0;
    for (int x = 0; x < kernelWidth; ++x) {
      sumA[1][y] += krn[width * y + x];
      sumB[1][y] += img[width * y + x];
    }
  }

  for (int t = -2; t <= 2; ++t) {
    crlX[2 + t] = 0;
    crlY[2 + t] = 0;
    for (int o = kernelWidth + min(0, t) - max(0, t) - 1; o >= 0; --o)
      crlX[2 + t] += abs(sumA[0][o + max(0, t)] - sumB[0][o + -min(0, t)]);
    for (int o = kernelHeight + min(0, t) - max(0, t) - 1; o >= 0; --o)
      crlY[2 + t] += abs(sumA[1][o + max(0, t)] - sumB[1][o + -min(0, t)]);

    crlX[2 + t] /= kernelWidth + min(0, t) - max(0, t);
    crlY[2 + t] /= kernelHeight + min(0, t) - max(0, t);
  }
}

void blockMatching(uint8_t* krn, int kernelWidth, int kernelHeight,
                   uint8_t* img, int width, int height, float* dx, float* dy) {
  int32_t crlX[5], crlY[5];

  blockCorrelationCurves(krn, kernelWidth, kernelHeight, img, width, height,
                         crlX, crlY);

  //	for(int i = 0; i < 5; ++i)	cout<<crlX[i]<<" ";	cout<<endl;
  //	for(int i = 0; i < 5; ++i)	cout<<crlY[i]<<" ";	cout<<endl;

  *dx = -crl_sym_curve(crlX) / 256.0f;
  *dy = -crl_sym_curve(crlY) / 256.0f;

  //	cout<<"X: "<<*dx<<endl;
  //	cout<<"Y: "<<*dy<<endl;
}

int32_t crl_sym_curve(int32_t* y) {
  int32_t idx_match_fine;
  int32_t idx_min_match = 0x0FFFFFFF;

  //        int i;
  int32_t num, denom, sign;
  int64_t num_n, num_p, denom_n, denom_p;

  // assume exact min is to the left of observed min (negative offset),
  // use formula for offset -0.5..0
  num_n = (int64_t)(y[1] - y[3]) * (y[0] - y[1] - y[2] + y[3]) +
          (int64_t)(y[4] - y[0]) * (y[3] - y[4]);
  denom_n = (int64_t)(y[0] - y[1]) * (y[0] - y[1]) +
            (int64_t)(y[2] - y[3]) * (y[2] - y[3]) +
            (int64_t)(y[3] - y[4]) * (y[3] - y[4]);

  // assume the offset is positive - calculate using the formula
  // for offset 0..0.5
  num_p = -(int64_t)(y[3] - y[1]) * (y[4] - y[3] - y[2] + y[1]) -
          (int64_t)(y[0] - y[4]) * (y[1] - y[0]);
  denom_p = (int64_t)(y[4] - y[3]) * (y[4] - y[3]) +
            (int64_t)(y[2] - y[1]) * (y[2] - y[1]) +
            (int64_t)(y[1] - y[0]) * (y[1] - y[0]);

  while ((denom_n > 0x000000007fffffff) || (denom_n < -0x000000007fffffff) ||
         (denom_p > 0x000000007fffffff) || (denom_p < -0x000000007fffffff) ||
         (num_n > 0x000000007fffffff) || (num_n < -0x000000007fffffff) ||
         (num_p > 0x000000007fffffff) || (num_p < -0x000000007fffffff)) {
    num_p >>= 4;
    denom_p >>= 4;
    num_n >>= 4;
    denom_n >>= 4;
  }

  num = num_n;
  denom = denom_n;
  // actually denominator is 2*denom, so 128 is used below instead of 256
  if ((num ^ denom) > 0) {
    // the offset is positive - use the formula for offset 0..0.5
    num = num_p;
    denom = denom_p;
  }

  // if both calculkations give different sign of offset - ignore them
  if ((((int32_t)num_p ^ (int32_t)denom_p) ^
       ((int32_t)num_n ^ (int32_t)denom_n)) < 0)
    num = 0;

  sign = 1;
  // if the result is negative
  if ((num ^ denom) < 0) sign = -1;

  // perform UINT calc to do proper rounding
  num = abs(num);
  denom = abs(denom);
  if (denom == 0) denom = 1;

  if (num >> 25) {  // if > 01ffffff - count for overflow
    denom /= 128;
    if (denom == 0) denom = 1;
    idx_match_fine = (num + denom / 2) / denom;
  } else {
    idx_match_fine = (num * 128 + denom / 2) / denom;
  }

  if (idx_match_fine > 150) {
    // if the found offset is >0.5, then the upper formulas gave result
    // which is out of the range the formulas are suitable for. This
    // means non-symetric curve around the minimum.
    // leave some reserve (check for >0.6, not for >0.5) because some
    // little noise could cause >0.5 in a curve which has offset 0.5.
    idx_match_fine = 0;
  }

  idx_match_fine *= sign;
  return idx_match_fine;
}

void bilinearSubpixel(uint8_t* krn, int kernelWidth, int kernelHeight,
                      uint8_t* img, int width, int height, float* dx,
                      float* dy) {
  //	printf("Kernel:\n");
  //	for(int y = 0; y < kernelWidth; ++y)
  //	{
  //		for(int x = 0; x < kernelHeight; ++x)
  //		{
  //			printf("%d\t", krn[y*width + x]);
  //		}
  //		printf("\n");
  //	}
  //	printf("Image:\n");
  //	for(int y = 0; y < kernelWidth; ++y)
  //	{
  //		for(int x = 0; x < kernelHeight; ++x)
  //		{
  //			printf("%d\t", img[y*width + x]);
  //		}
  //		printf("\n");
  //	}

  uint8_t k[kernelWidth * kernelHeight];
  uint32_t sum, minsum = ~(uint32_t)0;

  // printf("Cor\n");
  for (float kdy = -1.00f; kdy <= 1.00f; kdy += 0.05f) {
    // if(kdy == 0.0f) continue;
    for (float kdx = -1.00f; kdx <= 1.00f; kdx += 0.05f) {
      // if(kdx == 0.0f) continue;
      bilinearInterpolationOut(krn, k, kernelWidth, kernelHeight, width,
                               kernelWidth, kdx, kdy);
      //			printf("Kernel (%.2f, %.2f):\n", kdx, kdy);
      //			for(int y = 0; y < kernelWidth; ++y)
      //			{
      //				for(int x = 0; x < kernelHeight; ++x)
      //				{
      //					printf("%d\t", k[y*kernelWidth +
      // x]);
      //				}
      //				printf("\n");
      //			}

      sum = sad_fst(k, img, kernelWidth, kernelHeight, 0, width - kernelWidth);
      if (sum < minsum) {
        *dx = kdx;
        *dy = kdy;
        minsum = sum;
      }
      // printf("%u\t", sum);
    }
    // printf("\n");
  }
}
#include <algorithm>
#include <cmath>
#include <cstdlib>
void bilinearInterpolation(uint8_t* in, uint8_t* out, int w, int h, int inppl,
                           int outppl, float dx, float dy) {
  int ox = dx > 0 ? -1 : 1;
  int oy = dy > 0 ? -1 : 1;

  int kw = w - 1 + (dx >= 0 ? 1 : 0);
  int kh = h - 1 + (dy >= 0 ? 1 : 0);

  int sx = 1 + (dx <= 0 ? -1 : 0);
  int sy = 1 + (dy <= 0 ? -1 : 0);

  dx = fabs(dx);
  dy = fabs(dy);

  // Sides
  for (int y = 0; y < sy; ++y)
    for (int x = 0; x < w; ++x)
      out[y * outppl + x] = ((int32_t)in[y * inppl + x]) * (1.0f - dy) + 0.5f;

  for (int y = kh; y < h; ++y)
    for (int x = 0; x < w; ++x)
      out[y * outppl + x] = ((int32_t)in[y * inppl + x]) * (1.0f - dy) + 0.5f;

  for (int x = 0; x < sx; ++x)
    for (int y = 0; y < h; ++y)
      out[y * outppl + x] = ((int32_t)in[y * inppl + x]) * (1.0f - dx) + 0.5f;

  for (int x = kw; x < w; ++x)
    for (int y = 0; y < h; ++y)
      out[y * outppl + x] = ((int32_t)in[y * inppl + x]) * (1.0f - dx) + 0.5f;

  // Corners
  //	if(sy > 0 || sx > 0)
  //		out[0*w + 0] = ((int32_t)in[0*w + 0])*(1.0f - dy)*(1.0f - dx) +
  // 0.5f;
  //
  //	if(kh < h || sx > 0)
  //		out[h*w + 0] = ((int32_t)in[h*w + 0])*(1.0f - dy)*(1.0f - dx) +
  // 0.5f;
  //
  //	if(kh < h || kw < w)
  //		out[h*w + w - 1] = ((int32_t)in[h*w + w - 1])*(1.0f - dy)*(1.0f
  //- dx) + 0.5f;
  //
  //	if(sy > 0 || kw < w)
  //		out[0*w + w - 1] = ((int32_t)in[0*w + w - 1])*(1.0f - dy)*(1.0f
  //- dx) + 0.5f;

  for (int y = sy; y < kh; ++y) {
    for (int x = sx; x < kw; ++x) {
      out[y * outppl + x] =
          ((int32_t)in[(y + 00) * inppl + x + 00]) * (1.0f - dx) * (1.0f - dy) +
          ((int32_t)in[(y + 00) * inppl + x + ox]) * (dx) * (1.0f - dy) +
          ((int32_t)in[(y + oy) * inppl + x + 00]) * (1.0f - dx) * (dy) +
          ((int32_t)in[(y + oy) * inppl + x + ox]) * (dx) * (dy) + 0.5f;
    }
  }
}

void bilinearInterpolationOut(uint8_t* in, uint8_t* out, int w, int h,
                              int inppl, int outppl, float dx, float dy) {
  int ox = dx > 0 ? -1 : 1;
  int oy = dy > 0 ? -1 : 1;

  dx = fabs(dx);
  dy = fabs(dy);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      out[y * outppl + x] =
          ((int32_t)in[(y + 00) * inppl + x + 00]) * (1.0f - dx) * (1.0f - dy) +
          ((int32_t)in[(y + 00) * inppl + x + ox]) * (dx) * (1.0f - dy) +
          ((int32_t)in[(y + oy) * inppl + x + 00]) * (1.0f - dx) * (dy) +
          ((int32_t)in[(y + oy) * inppl + x + ox]) * (dx) * (dy) + 0.5f;
    }
  }
}
