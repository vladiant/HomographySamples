/*
 * Time.h
 *
 *  Created on: Nov 3, 2011
 *
 *  Here is defined the time data type used throughout the library.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus

namespace Test {

typedef int64_t Nanoseconds;
typedef Nanoseconds Time;

template <typename T>
T timeToSeconds(Time t);
template <typename T>
Time timeFromSeconds(T t);

template <typename T>
T timeToSeconds(Time t) {
  return T(t) / (1000 * 1000 * 1000);
}

template <typename T>
Time timeFromSeconds(T t) {
  return Time(t * (1000 * 1000 * 1000));
}

}  // namespace Test

#endif
