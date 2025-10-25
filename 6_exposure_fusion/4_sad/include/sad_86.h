/*
 * File:   sad_86.h
 *
 * Created on May 21, 2012, 1:29 PM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t sad_fst(const uint8_t *ptr1, const uint8_t *ptr2, int32_t size_x,
                 int32_t size_y, int32_t stride1, int32_t stride2);
uint32_t ssd_fst(const uint8_t *ptr1, const uint8_t *ptr2, int32_t size_x,
                 int32_t size_y, int32_t stride1, int32_t stride2);

#ifdef __cplusplus
}
#endif

