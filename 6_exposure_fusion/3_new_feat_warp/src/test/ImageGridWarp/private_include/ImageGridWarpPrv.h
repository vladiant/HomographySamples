/*
 * File:   cTImageGridWarp.h
 *
 * Created on April 23, 2012, 1:39 PM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef IGW_T
#define IGW_T float
#endif  // IGW_T

void imageGridWarpAffine(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

void imageGridWarpAffine2(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

void imageGridWarpBilinearFNearest(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T* g_x, uint32_t g_sx,   // X, x stride
    IGW_T* g_y, uint32_t g_sy,   // Y, y stride

    int temp);

void imageGridWarpBilinearFLUT(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T* g_x, uint32_t g_sx,   // X, x stride
    IGW_T* g_y, uint32_t g_sy,   // Y, y stride

    int temp);

#ifdef __cplusplus
}
#endif
