#include "sad_86.h"

#include <stdlib.h>

uint32_t sad_fst(const uint8_t *ptr1, const uint8_t *ptr2, int32_t size_x,
                 int32_t size_y, int32_t stride1, int32_t stride2) {
  int_fast32_t x, y;
  uint32_t sum = 0;

  for (y = 0; y < size_y; ++y) {
    for (x = 0; x < size_x; ++x) {
      sum += abs(*ptr1++ - *ptr2++);
    }
    ptr1 += stride1;
    ptr2 += stride2;
  }

  return sum;
}

uint32_t ssd_fst(const uint8_t *ptr1, const uint8_t *ptr2, int32_t size_x,
                 int32_t size_y, int32_t stride1, int32_t stride2) {
  int_fast32_t x, y;
  uint32_t sum = 0;

  for (y = 0; y < size_y; ++y) {
    for (x = 0; x < size_x; ++x) {
      sum += (*ptr1 - *ptr2) * (*ptr1 - *ptr2);
      ptr1++;
      ptr2++;
    }
    ptr1 += stride1;
    ptr2 += stride2;
  }

  return sum;
}
