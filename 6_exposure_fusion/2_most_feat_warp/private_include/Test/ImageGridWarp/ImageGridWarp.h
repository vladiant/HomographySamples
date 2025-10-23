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

/** @ingroup ImageGridWarp
 *  @{ */

/// \brief Image Forward Mapping Fixed Point Implementation
/// \param src The source Image data
/// \param src_w The width of the source image
/// \param src_h The height of the source image
/// \param src_ppl Source Image Pixels per Line
/// \param dst The destination Image data
/// \param dst_w The width of the destination image
/// \param dst_h The height of the destination image
/// \param dst_ppl Destination Image Pixels per Line
void imageGridWarpBilinearI(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

/// \brief Image Back Mapping Fixed Point Implementation
/// \param src The source Image data
/// \param src_w The width of the source image
/// \param src_h The height of the source image
/// \param src_ppl Source Image Pixels per Line
/// \param dst The destination Image data
/// \param dst_w The width of the destination image
/// \param dst_h The height of the destination image
/// \param dst_ppl Destination Image Pixels per Line
void imageGridWarpBilinearIBack(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

void imageGridWarpBilinearIBackGrayAlpha(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T* g_x, uint32_t g_sx,   // X, x stride
    IGW_T* g_y, uint32_t g_sy,   // Y, y stride

    int temp);

void imageGridWarpBilinearIBackGray(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r,  // Grid cols/rows
    IGW_T* g_x, uint32_t g_sx,   // X, x stride
    IGW_T* g_y, uint32_t g_sy,   // Y, y stride

    int temp);

/// \brief Image Forward Mapping Floating Point Implementation
void imageGridWarpBilinearF(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

/// \brief Image Back Mapping Floating Point Implementation
void imageGridWarpBilinearFBack(
    // Image
    void* src, uint32_t src_w, uint32_t src_h, uint32_t src_ppl, void* dst,
    uint32_t dst_w, uint32_t dst_h, uint32_t dst_ppl,

    // Warped Grid
    uint32_t g_c, uint32_t g_r, IGW_T* g_x, uint32_t g_sx,  // X, x stride
    IGW_T* g_y, uint32_t g_sy,                              // Y, y stride

    int temp);

/** @} */  // end of group1

#ifdef __cplusplus
}
#endif
