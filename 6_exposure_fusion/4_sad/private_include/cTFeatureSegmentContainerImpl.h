/*
 * File:   cTFeaturePointImpl.h
 *
 * Created on May 29, 2012, 5:00 PM
 */

#pragma once

#include <cassert>

#include "cTFeatureSegmentContainer.h"

template <typename coordT>
class cTFeatureSegmentContainer;

template <typename coordT>
cTFeatureSegmentContainer<coordT>::cTFeatureSegmentContainer() {
  initialize();
}

template <typename coordT>
template <typename coordT2>
cTFeatureSegmentContainer<coordT>::cTFeatureSegmentContainer(
    const cTFeatureSegmentContainer<coordT2> &orig) {
  initialize(orig);
}

template <typename coordT>
cTFeatureSegmentContainer<coordT>::cTFeatureSegmentContainer(
    const cTFeatureSegmentContainer<coordT> &orig) {
  initialize(orig);
}

template <typename coordT>
cTFeatureSegmentContainer<coordT>::~cTFeatureSegmentContainer() {
  if (segColsOld != 0 || segRowsOld != 0) {
    assert(points != 0);
    delete[] points;
    points = 0;
  }

  assert(points == 0);
}

template <typename coordT>
template <typename coordT2>
void cTFeatureSegmentContainer<coordT>::initialize(
    const cTFeatureSegmentContainer<coordT2> &orig) {
  // Typecast Deep Copy
  initialize();

  segCols = orig.segCols;
  segRows = orig.segRows;
  imgWidth = orig.imgWidth;
  imgHeight = orig.imgHeight;

  segChanged();

  assert(segCount == orig.segCount);
  for (uint32_t i = 0; i < orig.segCount; ++i) {
    points[i].x = static_cast<coordT>(orig.points[i].x);
    points[i].y = static_cast<coordT>(orig.points[i].y);
    points[i].status = orig.points[i].status;
  }
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::initialize(
    const cTFeatureSegmentContainer<coordT> &orig) {
  // Deep Copy
  initialize();

  segWidth = orig.segWidth;
  segHeight = orig.segHeight;
  segCols = orig.segCols;
  segRows = orig.segRows;
  segCount = orig.segCount;
  imgWidth = orig.imgWidth;
  imgHeight = orig.imgHeight;

  segChanged();

  assert(segCount == orig.segCount);
  memcpy(points, orig.points, sizeof(points[0]) * segCount);
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::initialize() {
  points = 0;
  segWidth = 0;
  segHeight = 0;
  segCols = 0;
  segRows = 0;
  segCount = 0;
  imgWidth = 0;
  imgHeight = 0;
  segColsOld = 0;
  segRowsOld = 0;
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::segChanged() {
  if (segCols != 0) {
    segWidth = imgWidth / segCols;
    if (segWidth * segCols < imgWidth) segWidth++;
    assert(segWidth * segCols >= imgWidth);
  }
  if (segRows != 0) {
    segHeight = imgHeight / segRows;
    if (segHeight * segRows < imgHeight) segHeight++;
    assert(segHeight * segRows >= imgHeight);
  }

  if (segCols != 0 && segRows != 0) allocate();
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::allocate() {
  if (segColsOld == segCols && segRowsOld == segRows) return;
  if (segColsOld != 0 || segRowsOld != 0) deallocate();

  segColsOld = segCols;
  segRowsOld = segRows;

  segCount = segCols * segRows;
  points = new cTFeaturePoint<coordT>[segCount];
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::deallocate() {
  assert(points != 0 && "Trying to double allocated memory!");

  delete[] points;

  segColsOld = 0;
  segRowsOld = 0;
  segCount = 0;
  points = 0;
}

template <typename coordT>
template <typename coordT2>
cTFeatureSegmentContainer<coordT> &cTFeatureSegmentContainer<coordT>::operator=(
    const cTFeatureSegmentContainer<coordT2> &orig) {
  initialize(orig);
  return *this;
}

template <typename coordT>
cTFeatureSegmentContainer<coordT> &cTFeatureSegmentContainer<coordT>::operator=(
    const cTFeatureSegmentContainer<coordT> &orig) {
  if (this == &orig) return *this;
  initialize(orig);
  return *this;
}

template <typename coordT>
cTFeaturePoint<coordT> &cTFeatureSegmentContainer<coordT>::operator()(
    uint32_t x, uint32_t y) {
  assert(y * segCols + x >= 0 && y * segCols + x < segCount &&
         "Index out of bounds");
  return points[y * segCols + x];
}

template <typename coordT>
cTFeaturePoint<coordT> &cTFeatureSegmentContainer<coordT>::operator[](
    size_t index) {
  assert(index >= 0 && index < segCount && "Index out of bounds");
  return points[index];
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::segmentSetCount(uint32_t cols,
                                                        uint32_t rows) {
  segCols = cols;
  segRows = rows;

  assert(segRows > 0 && "Segment Rows must be at least 1");
  assert(segCols > 0 && "Segment Columns must be at least 1");

  segChanged();
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::segmentGetWidth() const {
  return segWidth;
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::segmentGetHeight() const {
  return segHeight;
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::segmentGetCols() const {
  return segCols;
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::segmentGetRows() const {
  return segRows;
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::segmentGetCount() const {
  return segCount;
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::imageSetSize(uint32_t width,
                                                     uint32_t height) {
  imgWidth = width;
  imgHeight = height;
  assert(imgWidth > 8 && "Image Width  needs to be at least 9 pixels");
  assert(imgHeight > 8 && "Image Height needs to be at least 9 pixels");
  segChanged();
}

template <typename coordT>
void cTFeatureSegmentContainer<coordT>::imageResize(uint32_t width,
                                                    uint32_t height) {
  if (imgWidth == 0 || imgHeight == 0) {
    imageSetSize(width, height);
    return;
  }

  float xscale = width / (float)imgWidth;
  float yscale = height / (float)imgHeight;

  imageSetSize(width, height);

  for (uint32_t i = 0; i < segCount; ++i) {
    points[i].x *= xscale;
    points[i].y *= yscale;
  }
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::imageGetWidth() const {
  return imgWidth;
}

template <typename coordT>
uint32_t cTFeatureSegmentContainer<coordT>::imageGetHeight() const {
  return imgHeight;
}

