#pragma once

#ifndef AFF_DataT
#define AFF_DataT float  // Affine Type
#endif                   // AFF_DataT

#ifndef AFF_DataI
#define AFF_DataI int32_t  // Affine Type
#endif                     // AFF_DataT

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void findAffine(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                AFF_DataT x3, AFF_DataT y3, AFF_DataT X1, AFF_DataT Y1,
                AFF_DataT X2, AFF_DataT Y2, AFF_DataT X3, AFF_DataT Y3,
                AFF_DataT* H);

void findBilinear(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                  AFF_DataT x3, AFF_DataT y3, AFF_DataT x4, AFF_DataT y4,
                  AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                  AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                  AFF_DataT* out);
void findBilinear2(AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                   AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                   AFF_DataT x3, AFF_DataT y3, AFF_DataT x4, AFF_DataT y4,
                   AFF_DataT* out);
void findBilinear3(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                   AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                   AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                   AFF_DataT* out);
void findBilinear3O(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                    AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                    AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                    AFF_DataT* out);
void findBilinear3I(AFF_DataI x1, AFF_DataI y1, AFF_DataI x2, AFF_DataI y2,
                    AFF_DataI X1, AFF_DataI Y1, AFF_DataI X2, AFF_DataI Y2,
                    AFF_DataI X3, AFF_DataI Y3, AFF_DataI X4, AFF_DataI Y4,
                    AFF_DataI* out);

void findBilinearI(AFF_DataI x1, AFF_DataI y1, AFF_DataI x2, AFF_DataI y2,
                   AFF_DataI x3, AFF_DataI y3, AFF_DataI x4, AFF_DataI y4,
                   AFF_DataI X1, AFF_DataI Y1, AFF_DataI X2, AFF_DataI Y2,
                   AFF_DataI X3, AFF_DataI Y3, AFF_DataI X4, AFF_DataI Y4,
                   AFF_DataI* out);

#ifdef __cplusplus
}
#endif
