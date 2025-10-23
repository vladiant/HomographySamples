/*
 * File:   gamma.h
 * Author: vantonov
 *
 * Created on June 6, 2012, 10:28 AM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// A function to reverse gamma of image
void gamma_reverse(unsigned char* src, unsigned char* dst, int width,
                   int height, int ppl_src, int ppl_dst);

// A function to reverse gamma of gray image
void gamma_reverse_gray(unsigned char* src, unsigned char* dst, int width,
                        int height, int ppl_src, int ppl_dst);

// A function to convert image to grayscale and reverse gamma
void rgb24_to_gray_gamma_reverse(unsigned char* src, unsigned char* dst,
                                 int width, int height, int ppl_src,
                                 int ppl_dst);

#ifdef __cplusplus
}
#endif
