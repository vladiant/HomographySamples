/*
 * File:   klt.h
 *
 * Created on June 4, 2012, 2:17 PM
 */

#pragma once

#include <stdint.h>

void cone_fit(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
              int width, int height, float* coeffs);

void paraboloid_fit(uint8_t* kernel, int kernelWidth, int kernelHeight,
                    uint8_t* img, int width, int height, float* coeffs);

void klt_deriv(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
               int width, int height, float* dx, float* dy);

void blockCorrelationCurves(uint8_t* krn, int kernelWidth, int kernelHeight,
                            uint8_t* img, int width, int height, int32_t* crlX,
                            int32_t* crlY);

void blockMatching(uint8_t* krn, int kernelWidth, int kernelHeight,
                   uint8_t* img, int width, int height, float* dx, float* dy);

int32_t crl_sym_curve(int32_t* crl_curve);

void bilinearSubpixel(uint8_t* krn, int kernelWidth, int kernelHeight,
                      uint8_t* img, int width, int height, float* dx,
                      float* dy);

void bilinearInterpolation(uint8_t* in, uint8_t* out, int w, int h, int inppl,
                           int outppl, float dx, float dy);

void bilinearInterpolationOut(uint8_t* in, uint8_t* out, int w, int h,
                              int inppl, int outppl, float dx, float dy);

void sad_brute(uint8_t* kernel, int kernelWidth, int kernelHeight, uint8_t* img,
               int width, int height, char* out, int* minx, int* miny);

// template <typename outT>
void sad_brute(uint8_t* kernel, int kernelWidth, int kernelHeight, int sx,
               int sy, int swid, int shei, uint8_t* img, int width, int height,
               char* out, int* minx, int* miny);
