/*
 * File:   cTFeaturePoint.h
 *
 * Created on May 29, 2012, 4:18 PM
 */

#pragma once

#include <cstddef>
#include <cstdint>

template <typename coordT>
struct cTFeaturePoint {
  coordT x, y;
  uint8_t status;
  uint8_t score;
};

template <typename coordT>
class cTFeatureSegmentContainer {
  template <typename coordT2>
  friend class cTFeatureSegmentContainer;

 public:
  cTFeatureSegmentContainer();
  template <typename coordT2>
  cTFeatureSegmentContainer(const cTFeatureSegmentContainer<coordT2> &orig);
  cTFeatureSegmentContainer(const cTFeatureSegmentContainer<coordT> &orig);
  ~cTFeatureSegmentContainer();

  template <typename coordT2>
  cTFeatureSegmentContainer<coordT> &operator=(
      const cTFeatureSegmentContainer<coordT2> &orig);
  cTFeatureSegmentContainer<coordT> &operator=(
      const cTFeatureSegmentContainer<coordT> &orig);
  inline cTFeaturePoint<coordT> &operator[](size_t index);
  inline cTFeaturePoint<coordT> &operator()(uint32_t x, uint32_t y = 0);

  void segmentSetCount(uint32_t cols, uint32_t rows);
  inline uint32_t segmentGetWidth() const;
  inline uint32_t segmentGetHeight() const;
  inline uint32_t segmentGetCols() const;
  inline uint32_t segmentGetRows() const;
  inline uint32_t segmentGetCount() const;

  void imageSetSize(uint32_t width, uint32_t height);
  void imageResize(uint32_t width, uint32_t height);
  inline uint32_t imageGetWidth() const;
  inline uint32_t imageGetHeight() const;

 private:
  template <typename coordT2>
  void initialize(const cTFeatureSegmentContainer<coordT2> &orig);
  void initialize(const cTFeatureSegmentContainer<coordT> &orig);
  void initialize();
  void segChanged();
  void allocate();
  void deallocate();

  cTFeaturePoint<coordT> *points;
  uint32_t segWidth;
  uint32_t segHeight;
  uint32_t segCols;
  uint32_t segRows;
  uint32_t segCount;
  uint32_t imgWidth;
  uint32_t imgHeight;

  uint32_t segColsOld;
  uint32_t segRowsOld;
};

#include "cTFeatureSegmentContainerImpl.h"

