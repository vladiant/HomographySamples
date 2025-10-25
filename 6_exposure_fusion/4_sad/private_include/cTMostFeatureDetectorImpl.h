/*
 * File:   cTMostFeatureDetector.cpp
 *
 * Created on May 15, 2012, 1:16 PM
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;

template <typename coordT, int bpp>
class cTMostFeatureDetector;

template <typename coordT, int bpp>
cTMostFeatureDetector<coordT, bpp>::cTMostFeatureDetector() {
  patternWidth = 0;
  oldWidth = 0;
  oldHeight = 0;
}

template <typename coordT, int bpp>
cTMostFeatureDetector<coordT, bpp>::cTMostFeatureDetector(
    cTFeatureSegmentContainer<coordT>* icontainer) {
  patternWidth = 0;
  oldWidth = 0;
  oldHeight = 0;
  container = icontainer;
}

template <typename coordT, int bpp>
cTMostFeatureDetector<coordT, bpp>::~cTMostFeatureDetector() {}

template <typename coordT, int bpp>
void cTMostFeatureDetector<coordT, bpp>::removeCloseF(int distance) {
  // Elimination Type 1
  for (uint32_t y = 1; y < container->segmentGetRows() - 1; ++y) {
    for (uint32_t x = 0; x < container->segmentGetCols() - 1; ++x) {
      if ((*container)(x, y).status == 0) continue;

      if (fabs((*container)(x, y).x - (*container)(x + 1, y).x) +
              fabs((*container)(x, y).y - (*container)(x + 1, y).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y).score) {
          (*container)(x + 1, y).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (fabs((*container)(x, y).x - (*container)(x, y + 1).x) +
              fabs((*container)(x, y).y - (*container)(x, y + 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x, y + 1).score) {
          (*container)(x, y + 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (fabs((*container)(x, y).x - (*container)(x + 1, y + 1).x) +
              fabs((*container)(x, y).y - (*container)(x + 1, y + 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y + 1).score) {
          (*container)(x + 1, y + 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (fabs((*container)(x, y).x - (*container)(x + 1, y - 1).x) +
              fabs((*container)(x, y).y - (*container)(x + 1, y - 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y - 1).score) {
          (*container)(x + 1, y - 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
    }
  }
}

template <typename coordT, int bpp>
void cTMostFeatureDetector<coordT, bpp>::removeClose(int distance) {
  // Elimination Type 1
  for (uint32_t y = 1; y < container->segmentGetRows() - 1; ++y) {
    for (uint32_t x = 0; x < container->segmentGetCols() - 1; ++x) {
      if ((*container)(x, y).status == 0) continue;

      if (abs((*container)(x, y).x - (*container)(x + 1, y).x) +
              abs((*container)(x, y).y - (*container)(x + 1, y).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y).score) {
          (*container)(x + 1, y).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (abs((*container)(x, y).x - (*container)(x, y + 1).x) +
              abs((*container)(x, y).y - (*container)(x, y + 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x, y + 1).score) {
          (*container)(x, y + 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (abs((*container)(x, y).x - (*container)(x + 1, y + 1).x) +
              abs((*container)(x, y).y - (*container)(x + 1, y + 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y + 1).score) {
          (*container)(x + 1, y + 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
      if (abs((*container)(x, y).x - (*container)(x + 1, y - 1).x) +
              abs((*container)(x, y).y - (*container)(x + 1, y - 1).y) <
          distance) {
        if ((*container)(x, y).score > (*container)(x + 1, y - 1).score) {
          (*container)(x + 1, y - 1).status = 0;
        } else {
          (*container)(x, y).status = 0;
          continue;  // We our segment is removed, no point to continue
                     // comparisons
        }
      }
    }
  }
}

template <typename coordT, int bpp>
template <int img_bpp>
void cTMostFeatureDetector<coordT, bpp>::draw(unsigned char* img, int width,
                                              int height,
                                              const uint32_t color) {
  float stepx = width / (float)oldWidth;
  // float stepy = height / (float)oldHeight;

  for (uint32_t y = 0; y < container->segmentGetRows(); ++y) {
    for (uint32_t x = 0; x < container->segmentGetCols(); ++x) {
      if ((*container)(x, y).status == 1) {
        int xx = (*container)(x, y).x;
        int yy = (*container)(x, y).y;
        assert(xx >= 0 && xx < oldWidth && yy >= 0 && yy < oldHeight ||
               !(cerr << xx << " " << yy << " " << oldWidth << " " << oldHeight
                      << endl));
        for (int b = 0; b < img_bpp; ++b)
          img[(((int)(yy * stepx)) * width + (int)(xx * stepx)) * img_bpp + b] =
              ((uint8_t*)&color)[b];
      }
    }
  }
}

// 250us - 20 threshold
// 283us - segmentation
// 430us - threshold 5
// 270us - threshold 20
template <typename coordT, int bpp>
void cTMostFeatureDetector<coordT, bpp>::detect(const unsigned char* p2,
                                                int width, int height, int ppln,
                                                unsigned char threshold) {
  assert(container != 0 && "Container not Set!");

  initOffsetPattern(ppln);
  if (container == 0 || oldWidth != width || oldHeight != height) {
    oldWidth = width;
    oldHeight = height;
    container->imageSetSize(width, height);
  }

  int cb;
  int c_b;
  int_fast16_t offset0 = patternOffset[0];
  int_fast16_t offset1 = patternOffset[1];
  int_fast16_t offset2 = patternOffset[2];
  int_fast16_t offset3 = patternOffset[3];
  int_fast16_t offset4 = patternOffset[4];
  int_fast16_t offset5 = patternOffset[5];
  int_fast16_t offset6 = patternOffset[6];
  int_fast16_t offset7 = patternOffset[7];
  int_fast16_t offset8 = patternOffset[8];
  int_fast16_t offset9 = patternOffset[9];
  int_fast16_t offset10 = patternOffset[10];
  int_fast16_t offset11 = patternOffset[11];
  int_fast16_t offset12 = patternOffset[12];
  int_fast16_t offset13 = patternOffset[13];
  int_fast16_t offset14 = patternOffset[14];
  int_fast16_t offset15 = patternOffset[15];
  const unsigned char* p;

  size_t nl = (ppln - container->segmentGetWidth()) * bpp;

  // p += 3 + 3*ppln;
  // p--;

  for (uint32_t y = 0; y < container->segmentGetRows(); ++y)
    for (uint32_t x = 0; x < container->segmentGetCols(); ++x)
      (*container)(x, y).status = 0;

  // uint32_t seg_rows2 = container->segmentGetRows() - 4;
  uint32_t seg_rows2 =
      (container->imageGetHeight() / container->segmentGetHeight()) - 1;
  uint32_t seg_cols2 =
      (container->imageGetWidth() / container->segmentGetWidth()) - 1;

  //	cout<<container->segmentGetCols()<<" "<<container->segmentGetRows()<<"
  //"<<
  //container->segmentGetCols()*container->segmentGetWidth()<<"
  //"<<
  //container->segmentGetRows()*container->segmentGetHeight()<<"
  //"<< 			seg_rows2*container->segmentGetHeight()<<" "<<
  //			seg_cols2*container->segmentGetWidth()<<endl;

  for (uint32_t sy = 1; sy < seg_rows2; ++sy) {
    for (uint32_t sx = 1; sx < seg_cols2; ++sx) {
      int maxthres = threshold;
      int maxx = -1, maxy = -1;
      p = &p2[((sy * container->segmentGetHeight()) * ppln +
               sx * container->segmentGetWidth()) *
              bpp] -
          bpp;
      for (uint32_t y = 0; y < container->segmentGetHeight(); ++y) {
        for (uint32_t x = 0; x < container->segmentGetWidth(); ++x) {
          assert(sx * container->segmentGetWidth() + x - 3 >= 0 &&
                     sx * container->segmentGetWidth() + x + 3 < oldWidth &&
                     sy * container->segmentGetHeight() + y - 3 >= 0 &&
                     sy * container->segmentGetHeight() + y + 3 < oldHeight ||
                 !(cerr << sx * container->segmentGetWidth() + x << " "
                        << sy * container->segmentGetHeight() + y << endl));

          p += bpp;
          cb = *p + maxthres;
          c_b = *p - maxthres;
#include "cTMostFeatureDetectorOast.h"

          maxthres = cornerScore(p, patternOffset, maxthres) + 1;
          maxx = x;
          maxy = y;
        }
        p += nl;
      }
      if (maxx != -1 && maxy != -1) {
        (*container)(sx, sy).status = 1;
        (*container)(sx, sy).score = maxthres;
        (*container)(sx, sy).x = sx * container->segmentGetWidth() + maxx;
        (*container)(sx, sy).y = sy * container->segmentGetHeight() + maxy;
      }
    }
  }
}

template <typename coordT, int bpp>
void cTMostFeatureDetector<coordT, bpp>::initOffsetPattern(int img_width) {
  if (patternWidth == img_width) return;

  patternWidth = img_width;
  patternOffset[0] = ((-3) + (0) * patternWidth) * bpp;
  patternOffset[1] = ((-3) + (-1) * patternWidth) * bpp;
  patternOffset[2] = ((-2) + (-2) * patternWidth) * bpp;
  patternOffset[3] = ((-1) + (-3) * patternWidth) * bpp;
  patternOffset[4] = ((0) + (-3) * patternWidth) * bpp;
  patternOffset[5] = ((1) + (-3) * patternWidth) * bpp;
  patternOffset[6] = ((2) + (-2) * patternWidth) * bpp;
  patternOffset[7] = ((3) + (-1) * patternWidth) * bpp;
  patternOffset[8] = ((3) + (0) * patternWidth) * bpp;
  patternOffset[9] = ((3) + (1) * patternWidth) * bpp;
  patternOffset[10] = ((2) + (2) * patternWidth) * bpp;
  patternOffset[11] = ((1) + (3) * patternWidth) * bpp;
  patternOffset[12] = ((0) + (3) * patternWidth) * bpp;
  patternOffset[13] = ((-1) + (3) * patternWidth) * bpp;
  patternOffset[14] = ((-2) + (2) * patternWidth) * bpp;
  patternOffset[15] = ((-3) + (1) * patternWidth) * bpp;

  for (int i = 16; i < 25; i++) patternOffset[i] = patternOffset[i - 16];
}

template <typename coordT, int bpp>
int cTMostFeatureDetector<coordT, bpp>::cornerScore(const uint8_t* ptr,
                                                    const int_fast16_t pixel[],
                                                    int threshold) {
  const int K = 8, N = 16 + K + 1;
  int k, v = ptr[0];
  short d[N];
  for (k = 0; k < N; k++) d[k] = (short)(v - ptr[pixel[k]]);

  int a0 = threshold;
  for (k = 0; k < 16; k += 2) {
    int a = std::min((int)d[k + 1], (int)d[k + 2]);
    a = std::min(a, (int)d[k + 3]);
    if (a <= a0) continue;
    a = std::min(a, (int)d[k + 4]);
    a = std::min(a, (int)d[k + 5]);
    a = std::min(a, (int)d[k + 6]);
    a = std::min(a, (int)d[k + 7]);
    a = std::min(a, (int)d[k + 8]);
    a0 = std::max(a0, std::min(a, (int)d[k]));
    a0 = std::max(a0, std::min(a, (int)d[k + 9]));
  }

  int b0 = -a0;
  for (k = 0; k < 16; k += 2) {
    int b = std::max((int)d[k + 1], (int)d[k + 2]);
    b = std::max(b, (int)d[k + 3]);
    b = std::max(b, (int)d[k + 4]);
    b = std::max(b, (int)d[k + 5]);
    if (b >= b0) continue;
    b = std::max(b, (int)d[k + 6]);
    b = std::max(b, (int)d[k + 7]);
    b = std::max(b, (int)d[k + 8]);

    b0 = std::min(b0, std::max(b, (int)d[k]));
    b0 = std::min(b0, std::max(b, (int)d[k + 9]));
  }

  threshold = -b0 - 1;

  return threshold;
}

template <typename coordT, int bpp>
void cTMostFeatureDetector<coordT, bpp>::setContainer(
    cTFeatureSegmentContainer<coordT>* icontainer) {
  container = icontainer;
}

template <typename coordT, int bpp>
cTFeatureSegmentContainer<coordT>*
cTMostFeatureDetector<coordT, bpp>::getContainer() {
  return container;
}

