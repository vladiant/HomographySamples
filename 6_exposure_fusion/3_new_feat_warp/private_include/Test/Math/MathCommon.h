/*
 * Math.h
 *
 *  Created on: Nov 1, 2011
 */

#pragma once

#ifdef __cplusplus

#include <stdint.h>
#include <stdlib.h>

#include <climits>
#include <cmath>

#include "../CoreCommon.h"

namespace Test {
namespace Math {

template <typename T, unsigned Size>
struct Vector;

template <typename T>
T abs(const T& x);
template <typename T>
T sqrt(const T& x);
template <typename T>
T exp(const T& x);
template <typename T>
T ln(const T& x);
template <typename T>
T sin(const T& x);
template <typename T>
T cos(const T& x);
template <typename T>
T tan(const T& x);
template <typename T>
T acos(const T& x);
template <typename T>
T asin(const T& x);
template <typename T>
T atan(const T& x);
template <typename T>
T sign(const T& x);
bool equal(float a, float b, unsigned maxUlps = 100);
bool equal(double a, double b, unsigned maxUlps = 200);
bool equal(int a, int b, unsigned maxUlps = 100);

template <typename NumT, unsigned Size>
NumT signOfOrientedDistanceFromPointToPlane(
    const Vector<NumT, Size>& planeNormal,
    const Vector<NumT, Size>& pointInPlane, const Vector<NumT, Size>& point);

// Note: use only with unsigned types
template <typename T>
unsigned numberOfLeadingZeros(T x);

}  // namespace Math
}  // namespace Test

#include "Vector.h"

namespace Test {
namespace Math {

template <typename T>
T abs(const T& x) {
  return (x >= T(0) ? x : -x);
}

template <typename T>
T sqrt(const T& x) {
  return std::sqrt(x);
}

template <typename T>
T exp(const T& x) {
  return std::exp(x);
}

template <typename T>
T ln(const T& x) {
  return std::log(x);
}

template <typename T>
T sin(const T& x) {
  return std::sin(x);
}

template <typename T>
T cos(const T& x) {
  return std::cos(x);
}

template <typename T>
T tan(const T& x) {
  return std::tan(x);
}

template <typename T>
T acos(const T& x) {
  return std::acos(x);
}

template <typename T>
T asin(const T& x) {
  return std::asin(x);
}

template <typename T>
T atan(const T& x) {
  return std::atan(x);
}

template <typename T>
T sign(const T& x) {
  return x >= T(0) ? T(1) : T(-1);
}

inline bool equal(float a, float b, unsigned maxUlps) {
  // Make sure maxUlps is non-negative and small enough that the
  // default NAN won't compare as equal to anything.
  TEST_ALGORITHMS_CORE_ASSERT(maxUlps < 4 * 1024 * 1024);
  int32_t aInt;
  union CastUnion {
    float src;
    int32_t dst;
  };
  aInt = reinterpret_cast<CastUnion*>(&a)->dst;
  // Make aInt lexicographically ordered as a twos-complement int
  if (aInt < 0) {
    aInt = 0x80000000 - aInt;
  }
  // Make bInt lexicographically ordered as a twos-complement int
  int32_t bInt = reinterpret_cast<CastUnion*>(&b)->dst;
  if (bInt < 0) {
    bInt = 0x80000000 - bInt;
  }
  uint32_t intDiff = abs(aInt - bInt);
  if (intDiff <= maxUlps) {
    return true;
  }
  return false;
}

inline bool equal(double a, double b, unsigned maxUlps) {
  // Make sure maxUlps is non-negative and small enough that the
  // default NAN won't compare as equal to anything.
  TEST_ALGORITHMS_CORE_ASSERT(maxUlps < 4 * 1024 * 1024);
  // int64_t aInt = *reinterpret_cast<int64_t*>(reinterpret_cast<char*>(&a));
  int64_t aInt;
  union CastUnion {
    double src;
    int64_t dst;
  };
  aInt = reinterpret_cast<CastUnion*>(&a)->dst;

  // Make aInt lexicographically ordered as a twos-complement int
  if (aInt < 0) {
    aInt = (int64_t(1) << 63) - aInt;
  }
  // Make bInt lexicographically ordered as a twos-complement int
  int64_t bInt = reinterpret_cast<CastUnion*>(&b)->dst;
  if (bInt < 0) {
    bInt = (int64_t(1) << 63) - bInt;
  }
  uint64_t intDiff = llabs(aInt - bInt);
  if (intDiff <= maxUlps) {
    return true;
  }
  return false;
}

inline bool equal(int a, int b, unsigned maxUlps) {
  unsigned diff = abs(a - b);
  if (diff <= maxUlps) {
    return true;
  }
  return false;
}

template <typename NumT, unsigned Size>
NumT signOfOrientedDistanceFromPointToPlane(
    const Vector<NumT, Size>& planeNormal,
    const Vector<NumT, Size>& pointInPlane, const Vector<NumT, Size>& point) {
  return sign((planeNormal * pointInPlane) /
              (planeNormal * point - planeNormal * planeNormal));
}

template <typename T>
unsigned numberOfLeadingZeros(T x) {
  unsigned result = sizeof(T) * CHAR_BIT;

  while (x != 0) {
    x >>= 1;
    result--;
  }

  return result;
}

}  // namespace Math
}  // namespace Test

#endif
