#pragma once

#include <stdint.h>

#include "Test/Math/choleski.h"

//***************************************************************************
//***************************************************************************
//***************************************************************************
template <typename float_t>
int32_t choleski_solve(float_t* matr, float_t* vect, float_t* res,
                       uint8_t dimentionality, void* scratch) {
  // Solves linear system of type:
  // b = A * x
  //'x' is the unknown vector
  //'A' is a positive definite matrix (e.g. regression matrix)
  //'b' is a vector of the left sides of the linear equations
  // matr is lower-left triangle part of A,
  // represented in memory as: a00, a10, a11, a20, a21, a22, ...
  //
  //  !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!!
  //  the equations in A must be ordered in such a way, the elemenst in the
  //  diagonal of A are ordered in accending order from A[00] to A[dim,dim] This
  //  is necessary to improve calculations accuracy
  //  !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!!
  //
  // vect is the 'b'
  //'x' is written in res
  // dimentionality is the number of equations, i.e. the dimentionality of A,b,x
  // scratch is working memory of size:
  // scratch_size = ( dim^2 + 3*dim ) * sizeof(float_t)

  // A=LDL',
  // L is lower triangular matrix with diagonal 1s, L' is its tranpose
  // D is adiagonal matrix
  //           1   0   0     D0 0  0     1  L10 L20
  // A=LDL' = L10  1   0  *  0  D1 0  *  0   1  L21
  //          L20 L21  1     0  0 D2     0   0   0
  // Dj = Ajj - SUMk(Ljk^2 * Dk), k=0..j-1
  // Lij = 1/Dj * (Aij - SUMk(Lik * Ljk * Dk), k=0..j-1
  // i > j above, i is row number, j is col number, both = 0..dim-1
  // D0 = A00
  // L10 = 1/D0 * A10
  // D1 = A11 - L10^2*D0
  // So, upper formulas are used for i>=2
  // matr is 1dimensional array, lower triangular part, so Aij =
  // matr[(i+1)*i/2+j]

  float_t *d, *d1, *l, *bmid;  // d1 keeps 1/d[i]
  float_t* lrow;
  int32_t dim;
  int32_t i, j;

  dim = dimentionality;
  d = (float_t*)scratch;
  d1 = d + dim;
  bmid = d1 + dim;
  l = bmid + dim;

  d[0] = *matr++;
  if (d[0] == 0) {
    return -1;
  }
  d1[0] = 1.0 / d[0];
  l[dim] = (*matr++) * d1[0];
  d[1] = (*matr++) - l[dim] * l[dim] * d[0];
  if (d[1] == 0) {
    return -1;
  }
  d1[1] = 1.0 / d[1];

  for (i = 2; i < dim; i++) {
    float_t sumd;
    lrow = l + i * dim;
    lrow[0] = (*matr++) * d1[0];
    sumd = lrow[0] * lrow[0] * d[0];
    for (j = 1; j < i; j++) {
      int16_t k;
      float_t* lrowup = l + j * dim;
      float_t suml = 0;
      for (k = 0; k < j; k++) {
        suml += lrow[k] * lrowup[k] * d[k];  // TODO: lrow[j-1] -> (*lrow++)
      }
      lrow[j] = ((*matr++) - suml) * d1[j];  // TODO lrow[j] -> (*lrow)
      sumd += lrow[j] * lrow[j] * d[j];      // TODO lrow[j] -> (*lrow)
    }
    d[i] = (*matr++) - sumd;
    if (d[i] == 0) {
      return -1;
    }
    d1[i] = 1 / d[i];
  }
  // middle result b' = DL'x
  // b = (L * b')
  // first : b'[0] = b[0]
  // for j=1..dim-1
  // b'[j] = bj - SUM(Lik * b[k-1]), k=0..j-1
  bmid[0] = vect[0];
  for (j = 1; j < dim; j++) {
    float_t sum = 0;
    for (i = 0; i < j; i++) {
      sum += bmid[i] * l[j * dim + i];
    }
    bmid[j] = vect[j] - sum;
  }
  // middle result b" = L'x
  // b' = D*b"
  for (j = 0; j < dim; j++) {
    bmid[j] *= d1[j];
  }
  // final result
  // b" = L'x
  // last : x[dim-1] = b"[dim-1]
  // j=dim-2..0
  // x[j] = b"j - SUM(Lki * b"[k+1]) ,k = j+1..dim-1  //note L'[ik] = L[ki]
  res[dim - 1] = bmid[dim - 1];
  for (j = dim - 2; j >= 0; j--) {
    float_t sum = 0;
    for (i = j + 1; i < dim; i++) {
      sum += res[i] * l[i * dim + j];  // b'i*L'ji = b'i*Lij
    }
    res[j] = bmid[j] - sum;
  }
  return 0;
}
