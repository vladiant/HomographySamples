/*
 * File:   imgconvertion.h
 *
 * Created on May 4, 2012, 2:31 PM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void rgb24_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst);
void rgb24_to_rgb32(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst);
void rgb32_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst);

void rgb24_to_gray(unsigned char* src, unsigned char* dst, int width,
                   int height, int ppl_src, int ppl_dst);
void gray_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                   int height, int ppl_src, int ppl_dst);

void yuv422_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                     int height, int ppl_src, int ppl_dst);
void yuv422_to_gray(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst);
void rgb24_to_yuv422(unsigned char* src, unsigned char* dst, int width,
                     int height, int ppl_src, int ppl_dst);

#ifdef __cplusplus
}
#endif
