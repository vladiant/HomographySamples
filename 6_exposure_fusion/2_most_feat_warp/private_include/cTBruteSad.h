/*
 * File:   cTBruteSad.h
 *
 * Created on June 12, 2012, 4:13 PM
 */

#pragma once

#include <cTFeatureSegmentContainer.h>
#include <cTMostFeatureDetector.h>
#include <klt.h>
#include <sad_86.h>
#include <stdint.h>

#include <iostream>

void bruteForceSad(uint8_t* src, uint8_t* dst, int width, int height,
                   cTFeatureSegmentContainer<float>* features,
                   cTFeatureSegmentContainer<float>* motion, int kernelWidth,
                   int kernelHeight, int searchWidth, int searchHeight);
