#include "private_include/ImageGridWarpAffine.h"

#include <stdio.h>
#include <stdlib.h>

void findAffine(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                AFF_DataT x3, AFF_DataT y3, AFF_DataT X1, AFF_DataT Y1,
                AFF_DataT X2, AFF_DataT Y2, AFF_DataT X3, AFF_DataT Y3,
                AFF_DataT* H) {
  H[0] = ((y2 - y3) * X1 - (X2 - X3) * y1 + X2 * y3 - X3 * y2) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
  H[3] = ((y2 - y3) * Y1 - (Y2 - Y3) * y1 + Y2 * y3 - Y3 * y2) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
  H[1] = -((x2 - x3) * X1 - (X2 - X3) * x1 + X2 * x3 - X3 * x2) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
  H[4] = -((x2 - x3) * Y1 - (Y2 - Y3) * x1 + Y2 * x3 - Y3 * x2) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
  H[2] = ((x2 * y3 - x3 * y2) * X1 - (X2 * y3 - X3 * y2) * x1 +
          (X2 * x3 - X3 * x2) * y1) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
  H[5] = ((x2 * y3 - x3 * y2) * Y1 - (Y2 * y3 - Y3 * y2) * x1 +
          (Y2 * x3 - Y3 * x2) * y1) /
         ((y2 - y3) * x1 - (x2 - x3) * y1 + x2 * y3 - x3 * y2);
}

void findBilinear(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                  AFF_DataT x3, AFF_DataT y3, AFF_DataT x4, AFF_DataT y4,
                  AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                  AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                  AFF_DataT* out) {
  // printf("%.4f %.4f\t%.4f %.4f\t%.4f %.4f\t%.4f %.4f\n", x1, y1, x2, y2, x3,
  // y3, x4, y4); printf("%.4f %.4f\t%.4f %.4f\t%.4f %.4f\t%.4f %.4f\n", X1, Y1,
  // X2, Y2, X3, Y3, X4, Y4);

  out[0] =
      -((x3 * y4 - x4 * y3) * X2 - (X3 * y4 - X4 * y3) * x2 +
        (X3 * x4 - X4 * x3) * y2 -
        ((x3 - x4) * X2 - (X3 - X4) * x2 + X3 * x4 - X4 * x3) * y1 -
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * X1 +
        ((y3 - y4) * X2 - (X3 - X4) * y2 + X3 * y4 - X4 * y3) * x1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[4] =
      -((x3 * y4 - x4 * y3) * Y2 - (Y3 * y4 - Y4 * y3) * x2 +
        (Y3 * x4 - Y4 * x3) * y2 -
        ((x3 - x4) * Y2 - (Y3 - Y4) * x2 + Y3 * x4 - Y4 * x3) * y1 -
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * Y1 +
        ((y3 - y4) * Y2 - (Y3 - Y4) * y2 + Y3 * y4 - Y4 * y3) * x1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[1] =
      ((x3 * y4 - x4 * y4) * X2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           X1 +
       (X3 * x4 * y4 - X4 * x3 * y3 - (X3 * y4 - X4 * y3) * x2) * y2 +
       ((X3 - X4) * x2 * y2 - X3 * x4 * y4 + X4 * x3 * y3 -
        (x3 * y3 - x4 * y4) * X2 +
        ((y3 - y4) * X2 - (X3 - X4) * y2 + X3 * y4 - X4 * y3) * x1) *
           y1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[5] =
      ((x3 * y4 - x4 * y4) * Y2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           Y1 +
       (Y3 * x4 * y4 - Y4 * x3 * y3 - (Y3 * y4 - Y4 * y3) * x2) * y2 +
       ((Y3 - Y4) * x2 * y2 - Y3 * x4 * y4 + Y4 * x3 * y3 -
        (x3 * y3 - x4 * y4) * Y2 +
        ((y3 - y4) * Y2 - (Y3 - Y4) * y2 + Y3 * y4 - Y4 * y3) * x1) *
           y1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[2] =
      ((X3 * x4 - X4 * x3) * x2 * y2 -
       ((x3 - x4) * X2 - (X3 - X4) * x2 + X3 * x4 - X4 * x3) * x1 * y1 -
       (x3 * x4 * y3 - x3 * x4 * y4) * X2 - (X3 * x4 * y4 - X4 * x3 * y3) * x2 -
       ((X3 - X4) * x2 * y2 - X3 * x4 * y4 + X4 * x3 * y3 -
        (x3 * y3 - x4 * y4) * X2) *
           x1 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2) *
           X1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[6] =
      ((Y3 * x4 - Y4 * x3) * x2 * y2 -
       ((x3 - x4) * Y2 - (Y3 - Y4) * x2 + Y3 * x4 - Y4 * x3) * x1 * y1 -
       (x3 * x4 * y3 - x3 * x4 * y4) * Y2 - (Y3 * x4 * y4 - Y4 * x3 * y3) * x2 -
       ((Y3 - Y4) * x2 * y2 - Y3 * x4 * y4 + Y4 * x3 * y3 -
        (x3 * y3 - x4 * y4) * Y2) *
           x1 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2) *
           Y1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[3] =
      (((x3 * y4 - x4 * y4) * x2 * y3 -
        (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2) *
           X1 -
       ((x3 * y4 - x4 * y4) * X2 * y3 +
        (X3 * x4 * y4 - X4 * x3 * y3 - (X3 * y4 - X4 * y3) * x2) * y2) *
           x1 -
       ((X3 * x4 - X4 * x3) * x2 * y2 - (x3 * x4 * y3 - x3 * x4 * y4) * X2 -
        (X3 * x4 * y4 - X4 * x3 * y3) * x2 -
        ((x3 * y4 - x4 * y3) * X2 - (X3 * y4 - X4 * y3) * x2 +
         (X3 * x4 - X4 * x3) * y2) *
            x1) *
           y1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
  out[7] =
      (((x3 * y4 - x4 * y4) * x2 * y3 -
        (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2) *
           Y1 -
       ((x3 * y4 - x4 * y4) * Y2 * y3 +
        (Y3 * x4 * y4 - Y4 * x3 * y3 - (Y3 * y4 - Y4 * y3) * x2) * y2) *
           x1 -
       ((Y3 * x4 - Y4 * x3) * x2 * y2 - (x3 * x4 * y3 - x3 * x4 * y4) * Y2 -
        (Y3 * x4 * y4 - Y4 * x3 * y3) * x2 -
        ((x3 * y4 - x4 * y3) * Y2 - (Y3 * y4 - Y4 * y3) * x2 +
         (Y3 * x4 - Y4 * x3) * y2) *
            x1) *
           y1) /
      ((x3 * y4 - x4 * y4) * x2 * y3 -
       ((x3 * y4 - x4 * y4) * y3 + ((y3 - y4) * x2 - x3 * y3 + x4 * y4) * y2) *
           x1 -
       (x3 * x4 * y3 - x3 * x4 * y4 + (x3 * y4 - x4 * y3) * x2) * y2 +
       ((x3 - x4) * x2 * y2 + x3 * x4 * y3 - x3 * x4 * y4 -
        (x3 * y3 - x4 * y4) * x2 +
        ((y3 - y4) * x2 - (x3 - x4) * y2 + x3 * y4 - x4 * y3) * x1) *
           y1);
}

void findBilinear2(AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                   AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                   AFF_DataT x3, AFF_DataT y3, AFF_DataT x4, AFF_DataT y4,
                   AFF_DataT* out) {
  float S0, S1, S2, S3, S4, S5;

  S0 = (x2 * y2 - x1 * y1);
  S1 = ((y3 - y1) * S0 - (y2 - y1) * (x3 * y3 - x1 * y1));
  S2 = ((x3 - x1) * S0 - (x2 - x1) * (x3 * y3 - x1 * y1));
  S3 = (X2 - X1) * (x3 * y3 - x1 * y1);
  S4 = ((x4 - x1) * S0 - (x2 - x1) * (x4 * y4 - x1 * y1));
  S5 = ((y4 - y1) * S0 - (y2 - y1) * (x4 * y4 - x1 * y1));

  printf("S0 = %f\n", S0);
  printf("S1 = %f\n", S1);
  printf("S2 = %f\n", S2);
  printf("S3 = %f\n", S3);
  printf("S4 = %f\n", S4);
  printf("S5 = %f\n", S5);

  out[2] = S3 * S4 -
           (X2 - X1) * ((x4 * y4 - x1 * y1) - S0) * S2 / (S5 * S2 - S1 * S4);
  out[1] = (-S3 - out[2] * S1) / S2;
  out[0] = (-out[1] * (x2 - x1) - out[2] * (y2 - y1) - X1 + X2) / S0;
  out[3] = -out[0] * x1 * y1 - out[1] * x1 - out[2] * y1 + X1;

  printf("out[2] = %f\n", (S5 * S2 - S1 * S4));

  S0 = (x2 * y2 - x1 * y1);
  S1 = (x1 - x2) * (x3 * y3 - x1 * y1) + (x3 - x1) * S0;
  S2 = (y1 - y2) * (x3 * y3 - x1 * y1) + (y3 - y1) * S0;
  S3 = (x4 * y4 - x1 * y1) * (x1 - x2) + (x4 - x1) * S0;
  S4 = (x4 * y4 - x1 * y1) * (y1 - y2) - (y4 - y1) * S0;
  out[6] = S0 * (Y1 - Y2) * (S3 - S1) / (S4 * S1 - S3 * S2);
  out[5] = (-(Y1 - Y2) * S0 - out[6] * S2) / S1;
  out[4] = (out[5] * (x1 - x2) + out[6] * (y1 - y2)) / S0;
  out[7] = Y1 - out[4] * x1 * y1 - out[5] * x1 - out[6] * y1;
}

void findBilinear3(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                   AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                   AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                   AFF_DataT* out) {
  out[0] = (X1 - X2 - X3 + X4) / (x1 * y1 - x2 * y1 - x1 * y2 + x2 * y2);
  out[1] = (out[0] * (x2 * y1 - x1 * y1) + X1 - X2) / (x1 - x2);
  out[2] = (out[0] * (x1 * y2 - x1 * y1) + X1 - X3) / (y1 - y2);
  out[3] = -out[0] * x1 * y1 - out[1] * x1 - out[2] * y1 + X1;

  out[4] = (Y1 - Y2 - Y3 + Y4) / (x1 * y1 - x2 * y1 - x1 * y2 + x2 * y2);
  out[5] = (-out[4] * (x2 * y1 - x1 * y1) - Y1 + Y2) / (x2 - x1);
  out[6] = (-out[4] * (x1 * y2 - x1 * y1) - Y1 + Y3) / (y2 - y1);
  out[7] = -out[4] * x1 * y1 - out[5] * x1 - out[6] * y1 + Y1;
}

void findBilinear3I(AFF_DataI x1, AFF_DataI y1, AFF_DataI x2, AFF_DataI y2,
                    AFF_DataI X1, AFF_DataI Y1, AFF_DataI X2, AFF_DataI Y2,
                    AFF_DataI X3, AFF_DataI Y3, AFF_DataI X4, AFF_DataI Y4,
                    AFF_DataI* out) {
  // 0 1 2 # 4 5 6 # - 1<<25
  // # # # 3 # # # 7 - 1<<16

  // 25 = a / b16
  out[0] = (((int64_t)X1 - X2 - X3 + X4) << 32) /
           ((int64_t)x1 * y1 - (int64_t)x2 * y1 - (int64_t)x1 * y2 +
            (int64_t)x2 * y2);
  out[1] = (+out[0] * (((int64_t)x2 * y1 - (int64_t)x1 * y1) >> 16) +
            (((int64_t)X1 - X2) << 16)) /
           ((int64_t)x1 - x2);
  out[2] = (+out[0] * (((int64_t)x1 * y2 - (int64_t)x1 * y1) >> 16) +
            (((int64_t)X1 - X3) << 16)) /
           ((int64_t)y1 - y2);
  out[3] = ((-((int64_t)out[0] * x1 >> 16) * y1 - (int64_t)out[1] * x1 -
             (int64_t)out[2] * y1) >>
            16) +
           X1;

  out[4] = (((int64_t)Y1 - Y2 - Y3 + Y4) << 32) /
           ((int64_t)x1 * y1 - (int64_t)x2 * y1 - (int64_t)x1 * y2 +
            (int64_t)x2 * y2);
  out[5] = (-out[4] * (((int64_t)x2 * y1 - (int64_t)x1 * y1) >> 16) -
            (((int64_t)Y1 - Y2) << 16)) /
           ((int64_t)x2 - x1);
  out[6] = (-out[4] * (((int64_t)x1 * y2 - (int64_t)x1 * y1) >> 16) -
            (((int64_t)Y1 - Y3) << 16)) /
           ((int64_t)y2 - y1);
  out[7] = ((-((int64_t)out[4] * x1 >> 16) * y1 - (int64_t)out[5] * x1 -
             (int64_t)out[6] * y1) >>
            16) +
           Y1;
}

void findBilinear3O(AFF_DataT x1, AFF_DataT y1, AFF_DataT x2, AFF_DataT y2,
                    AFF_DataT X1, AFF_DataT Y1, AFF_DataT X2, AFF_DataT Y2,
                    AFF_DataT X3, AFF_DataT Y3, AFF_DataT X4, AFF_DataT Y4,
                    AFF_DataT* out) {
  AFF_DataT S1 = (x1 * y1 - x2 * y1 - x1 * y2 + x2 * y2);
  AFF_DataT S2 = x2 * y1 - x1 * y1;
  AFF_DataT S3 = x1 * y2 - x1 * y1;

  out[0] = (X1 - X2 - X3 + X4) / S1;
  out[1] = (out[0] * S2 + X1 - X2) / (x1 - x2);
  out[2] = (out[0] * S3 + X1 - X3) / (y1 - y2);
  out[3] = -out[0] * x1 * y1 - out[1] * x1 - out[2] * y1 + X1;

  out[4] = (Y1 - Y2 - Y3 + Y4) / S1;
  out[5] = (-out[4] * S2 - Y1 + Y2) / (x2 - x1);
  out[6] = (-out[4] * S3 - Y1 + Y3) / (y2 - y1);
  out[7] = -out[4] * x1 * y1 - out[5] * x1 - out[6] * y1 + Y1;
}
