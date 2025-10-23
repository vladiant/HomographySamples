/*
 * File:   cTBruteSad.cpp
 *
 * Created on June 12, 2012, 4:13 PM
 */

#include "cTBruteSad.h"

void bruteForceSad(uint8_t* src, uint8_t* dst, int width, int height,
                   cTFeatureSegmentContainer<float>* features,
                   cTFeatureSegmentContainer<float>* motion, int kernelWidth,
                   int kernelHeight, int searchWidth, int searchHeight) {
  *motion = *features;
  for (int i = 0; i < (*motion).segmentGetCount(); ++i) {
    if ((*motion)[i].status == 0) continue;

    int minx, miny;
    sad_brute(src + ((int)(*motion)[i].y - kernelHeight / 2) * width +
                  (int)(*motion)[i].x - kernelWidth / 2,
              kernelWidth, kernelHeight,
              (int)(*motion)[i].x - kernelWidth / 2 - searchWidth / 2,
              (int)(*motion)[i].y - kernelHeight / 2 - searchHeight / 2,
              searchWidth, searchHeight, dst, width, height, 0, &minx, &miny);

    (*motion)[i].x = minx + kernelWidth / 2;
    (*motion)[i].y = miny + kernelHeight / 2;
  }
}
