/*
 * File:   cTMostFeatureDetector.h
 *
 * Created on May 15, 2012, 1:16 PM
 */

#pragma once

#include <stdint.h>

#include "cTFeatureSegmentContainer.h"

template <typename coordT, int bpp = 1>
class cTMostFeatureDetector {
 public:
  cTMostFeatureDetector();
  cTMostFeatureDetector(cTFeatureSegmentContainer<coordT>* container);
  ~cTMostFeatureDetector();

  void detect(const unsigned char* img, int width, int height, int ppln,
              unsigned char threshold);

  template <int img_bpp>
  void draw(unsigned char* img, int width, int height, const uint32_t color);

  void removeClose(int distance);
  void removeCloseF(int distance);

  inline void setContainer(cTFeatureSegmentContainer<coordT>* container);
  inline cTFeatureSegmentContainer<coordT>* getContainer();

 private:
  void initOffsetPattern(int img_width);
  int cornerScore(const uint8_t* ptr, const int_fast16_t pixel[],
                  int threshold);

  cTFeatureSegmentContainer<coordT>* container;

  int oldWidth, oldHeight;

  int patternWidth;
  int_fast16_t patternOffset[16 + 9];
};

#include "cTMostFeatureDetectorImpl.h"
