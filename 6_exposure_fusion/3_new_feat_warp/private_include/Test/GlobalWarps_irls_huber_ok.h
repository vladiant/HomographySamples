/*
 * GlobalWarps.h
 *
 *  Created on: Apr 12, 2012
 */

// Affine transform
//  **********************************  x' = ax + by + c     Regresssion
//  **********************************  y' = dx + ey + f     Min Err
//[x y 1] * [a d] = [x' y']
//           [b e]
//           [c f]

// Perspective transform
// x3 = ax + by + c
// y3 = dx + ey + f
// z3 = gx + hy + 1
//
// x' = x3/z3
// y' = y3/z3
// Prime denotes resulting point, lack of prime denotes source point.

#pragma once

#include <vector>

#include "Test/Math/choleski.h"

namespace Test {
namespace GyroVStab {
#define TEST_GYRO_V_STAB_AFF_DIM 6
#define TEST_GYRO_V_STAB_PERSP_DIM 8
#define TEST_GYRO_V_STAB_TRANSF_MAX_DIM TEST_GYRO_V_STAB_PERSP_DIM

// Vertex is the type of vertex points.
// F is the type for floating point calculations.
// Dim is the dimentionality of the problem. It is equal to the number of
// coefficients.
template <typename Vertex, typename F>
class GlobalWarps {
 public:
  GlobalWarps();  // used for true (non-iterative) regressions
  //    GlobalWarps(std::vector<F> &deltas);     //used for iterative procedures

  // Computes transformation coefficients to move a straight grid into the
  // target grid.
  bool regAffine(Vertex *grid, unsigned hSize, unsigned vSize);
  bool regPersp(Vertex *grid, unsigned hSize, unsigned vSize);

  bool regPersp(Vertex *before_points, Vertex *after_points, unsigned vertCnt);
  // bool regAffine(Vertex *before_points, Vertex *after_points);

  F *getCoeffs();
  void warp(Vertex *target, unsigned hSize,
            unsigned vSize);  // in-place operation
  bool nowarp(Vertex *verts, unsigned hSize,
              unsigned vSize);  // Generates straight grid

 private:
  typedef enum {
    failedDetZero,
    succeededBigChange,
    succeededSmallChange,
  } SolveReturnCode;

  void composeMatAndVectPersp(Vertex *before, Vertex *after, unsigned vertCnt,
                              F g, F h, F *mat, F *vect);
  void composeMatAndVectPersp(Vertex *before, Vertex *after, unsigned vertCnt,
                              F g, F h, F *mat, F *vect, float weights[]);
  void composeMatAndVectAffine(Vertex *before, Vertex *after, unsigned vertCnt,
                               F *mat, F *vect1, F *vect2);
  SolveReturnCode solvePersp(F *mat, F *vect);
  SolveReturnCode solveAffine(F *mat, F *vect1, F *vect2);

  bool m_initialized;
  F mp_scratch[TEST_GYRO_V_STAB_TRANSF_MAX_DIM *
                   TEST_GYRO_V_STAB_TRANSF_MAX_DIM +
               3 * TEST_GYRO_V_STAB_TRANSF_MAX_DIM];
  F mp_coeffs[TEST_GYRO_V_STAB_TRANSF_MAX_DIM];
  std::vector<F> m_deltas;
  //    std::vector<F> m_deltas(TEST_GYRO_V_STAB_TRANSF_MAX_DIM);
};

////////////////////////// Implementation //////////////////////////////////

template <typename Vertex, typename F>
GlobalWarps<Vertex, F>::GlobalWarps()
    : m_deltas(TEST_GYRO_V_STAB_PERSP_DIM, 0.001) {}

// template <typename Vertex, typename F>
// GlobalWarps<Vertex, F>::GlobalWarps(std::vector<F> &deltas)
//     : m_deltas(deltas)
//{
//
// }

template <typename Vertex, typename F>
bool GlobalWarps<Vertex, F>::regPersp(Vertex *after, unsigned hSize,
                                      unsigned vSize) {
  F *it = mp_coeffs;
  *it++ = 1;  // a
  *it++ = 0;  // b
  *it++ = 0;  // c
  *it++ = 0;  // d
  *it++ = 1;  // e
  *it++ = 0;  // f
  *it++ = 0;  // g
  *it++ = 0;  // h

  F mat[(TEST_GYRO_V_STAB_PERSP_DIM * (TEST_GYRO_V_STAB_PERSP_DIM + 1)) / 2];
  F vect[TEST_GYRO_V_STAB_PERSP_DIM];
  SolveReturnCode ret;
  Vertex before[hSize * vSize];
  if (nowarp(before, hSize, vSize) != 0) {
    return false;
  }

  do {
    composeMatAndVectPersp(before, after, hSize * vSize, mp_coeffs[6],
                           mp_coeffs[7], mat, vect);
    ret = solvePersp(mat, vect);
    if (ret == failedDetZero) {
      return false;
    } else if (ret == succeededSmallChange) {
      return true;
    }
    // else     if(ret == succeededBigChange)
    // do next iteration
  } while (true);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename F>
bool GlobalWarps<Vertex, F>::regPersp(Vertex *before_points,
                                      Vertex *after_points, unsigned vertCnt) {
  F *it = mp_coeffs;
  *it++ = 1;  // a
  *it++ = 0;  // b
  *it++ = 0;  // c
  *it++ = 0;  // d
  *it++ = 1;  // e
  *it++ = 0;  // f
  *it++ = 0;  // g
  *it++ = 0;  // h

  // robust weights
  float weights[vertCnt];
  for (int i = 0; i < vertCnt; i++) weights[i] = 1;

  F mat[(TEST_GYRO_V_STAB_PERSP_DIM * (TEST_GYRO_V_STAB_PERSP_DIM + 1)) / 2];
  F vect[TEST_GYRO_V_STAB_PERSP_DIM];
  SolveReturnCode ret;

  unsigned iter = 0;
  int olderrcnt = vertCnt;
  bool flag = false;

  do {
    composeMatAndVectPersp(before_points, after_points, vertCnt, mp_coeffs[6],
                           mp_coeffs[7], mat, vect, weights);
    ret = solvePersp(mat, vect);

    // calculation of weights
    float errc = 0.0;
    float error = 0.0;
    float std = 0.0;

    double err[vertCnt];
    for (int i = 0; i < vertCnt; i++) err[i] = 0.0;

    int errcnt = 0;
    for (int o = 0; o < vertCnt; o++) {
      double dx_tmp = (before_points[o].x() * mp_coeffs[0] +
                       before_points[o].y() * mp_coeffs[1] + mp_coeffs[2]);
      double dy_tmp = (before_points[o].x() * mp_coeffs[3] +
                       before_points[o].y() * mp_coeffs[4] + mp_coeffs[5]);
      double dz_tmp = (before_points[o].x() * mp_coeffs[6] +
                       before_points[o].y() * mp_coeffs[7] + 1);

      errc = ((dx_tmp / dz_tmp) - after_points[o].x()) *
                 ((dx_tmp / dz_tmp) - after_points[o].x()) +
             ((dy_tmp / dz_tmp) - after_points[o].y()) *
                 ((dy_tmp / dz_tmp) - after_points[o].y());

      errc *= weights[o];
      errcnt += weights[o];
      error += errc;
      err[o] = errc;
    }
    std = error / vertCnt;

    std::cout << std << "  " << errcnt << std::endl;

    // ****************************** REMOVE OUTLIERS **********************
    for (int o = 0; o < vertCnt; o++) {
      if (err[o] > 1.345 * 1.345 * std) {
        weights[o] = 1.345 * 1.345 * std / (err[o]);
      } else
        weights[o] = 1.0;
    }

    for (int i = 0; i < TEST_GYRO_V_STAB_TRANSF_MAX_DIM; i++)
      std::cout << mp_coeffs[i] << "  ";
    std::cout << std::endl;

    if (ret == failedDetZero) {
      return false;
    } else if (ret == succeededSmallChange) {
      // if ((errcnt == olderrcnt)&&(flag)) return true;
    }
    // else     if(ret == succeededBigChange)
    // do next iteration

    olderrcnt = errcnt;
    if (!flag) flag = true;
    iter++;
    if (iter > 20) break;

  } while (true);
}

template <typename Vertex, typename F>
bool GlobalWarps<Vertex, F>::regAffine(Vertex *after, unsigned hSize,
                                       unsigned vSize) {
  F mat[((TEST_GYRO_V_STAB_AFF_DIM / 2) * (TEST_GYRO_V_STAB_AFF_DIM / 2 + 1)) /
        2];
  F vect1[TEST_GYRO_V_STAB_AFF_DIM / 2];
  F vect2[TEST_GYRO_V_STAB_AFF_DIM / 2];
  SolveReturnCode ret;
  Vertex before[hSize * vSize];

  mp_coeffs[6] = 0;  // g
  mp_coeffs[7] = 0;  // h

  if (nowarp(before, hSize, vSize) != 0) {
    return false;
  }

  composeMatAndVectAffine(before, after, hSize * vSize, mat, vect1, vect2);
  ret = solveAffine(mat, vect1, vect2);
  if (ret == failedDetZero) {
    return false;
  } else {
    return true;
  }
}

template <typename Vertex, typename F>
inline F *GlobalWarps<Vertex, F>::getCoeffs() {
  return mp_coeffs;
}

template <typename Vertex, typename F>
void GlobalWarps<Vertex, F>::warp(Vertex *target, unsigned hSize,
                                  unsigned vSize) {
  unsigned vertsCnt = hSize * vSize;
  Vertex p3;
  F z;

  nowarp(target, hSize, vSize);
  for (unsigned u = 0; u < vertsCnt; u++) {
    p3.x() = mp_coeffs[0] * target[u].x() + mp_coeffs[1] * target[u].y() +
             mp_coeffs[2];
    p3.y() = mp_coeffs[3] * target[u].x() + mp_coeffs[4] * target[u].y() +
             mp_coeffs[5];
    z = mp_coeffs[6] * target[u].x() + mp_coeffs[7] * target[u].y() + 1;

    target[u].x() = p3.x() / z;
    target[u].y() = p3.y() / z;
  }
}

template <typename Vertex, typename F>
bool GlobalWarps<Vertex, F>::nowarp(Vertex *verts, unsigned hSize,
                                    unsigned vSize) {
  if (hSize < 2 || vSize < 2) {
    return -1;
  }

  const F startX = -1;
  const F startY = -1;
  const F endX = 1;
  const F endY = 1;

  F stepX = (endX - startX) / (hSize - 1);
  F stepY = (endY - startY) / (vSize - 1);

  Vertex *it = verts;
  for (unsigned i = 0; i < vSize; i++) {
    for (unsigned j = 0; j < hSize; j++) {
      it->x() = startX + j * stepX;
      it->y() = startY + i * stepY;
      it++;
    }
  }
  return 0;
}

template <typename Vertex, typename F>
void GlobalWarps<Vertex, F>::composeMatAndVectPersp(Vertex *before,
                                                    Vertex *after,
                                                    unsigned vertCnt, F g, F h,
                                                    F *mat, F *vect) {
  // allocate space
  int N = 0;
  F w;
  F x, y;
  F xt, yt;
  F Sx2 = 0.0, Sxy = 0.0, Sy2 = 0.0, Sx = 0.0, Sy = 0.0, Smx2xt = 0.0,
    Smxyxt = 0.0, Smxxt = 0, Smx2yt = 0.0, Smxyyt = 0.0, Smxyt = 0.0,
    Sx2xt2px2yt2 = 0.0, Smy2xt = 0.0, Smyxt = 0.0, Smy2yt = 0.0, Smyyt = 0.0,
    Sxyxt2pxyyt2 = 0.0, Sy2xt2py2yt2 = 0;
  F Sxxt = 0.0, Syxt = 0.0, Sxt = 0.0, Sxyt = 0.0, Syyt = 0.0, Syt = 0.0,
    Smxxt2pxyt2 = 0.0, Smyxt2pyyt2 = 0.0;
  F *itMat = mat;
  F *itVect = vect;

  // generate sums
  for (unsigned o = 0; o < vertCnt; o++) {
    x = before[o].x();
    xt = after[o].x();
    y = before[o].y();
    yt = after[o].y();
    w = 1.0 / (g * x + h * y + 1);

    Sx2 += w * x * x;
    Sxy += w * x * y;
    Sy2 += w * y * y;
    Sx += w * x;
    Sy += w * y;
    ++N;
    Smx2xt += -w * x * x * xt;
    Smxyxt += -w * x * y * xt;
    Smxxt += -w * x * xt;
    Smx2yt += -w * x * x * yt;
    Smxyyt += -w * x * y * yt;
    Smxyt += -w * x * yt;
    Sx2xt2px2yt2 += w * x * x * (xt * xt + yt * yt);
    Smy2xt += -w * y * y * xt;
    Smyxt += -w * y * xt;
    Smy2yt += -w * y * y * yt;
    Smyyt += -w * y * yt;
    Sxyxt2pxyyt2 += w * (x * y) * (xt * xt + yt * yt);
    Sy2xt2py2yt2 += w * y * y * (xt * xt + yt * yt);

    Sxxt += w * x * xt;
    Syxt += w * y * xt;
    Sxt += w * xt;
    Sxyt += w * x * yt;
    Syyt += w * y * yt;
    Syt += w * yt;
    Smxxt2pxyt2 += -w * x * (xt * xt + yt * yt);
    Smyxt2pyyt2 += -w * y * (xt * xt + yt * yt);
  }

  // fill matrix
  *itMat++ = Sx2;
  *itMat++ = Sxy;
  *itMat++ = Sy2;
  *itMat++ = Sx;
  *itMat++ = Sy;
  *itMat++ = N;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sx2;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sxy;
  *itMat++ = Sy2;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sx;
  *itMat++ = Sy;
  *itMat++ = N;

  *itMat++ = Smx2xt;
  *itMat++ = Smxyxt;
  *itMat++ = Smxxt;
  *itMat++ = Smx2yt;
  *itMat++ = Smxyyt;
  *itMat++ = Smxyt;
  *itMat++ = Sx2xt2px2yt2;

  *itMat++ = Smxyxt;
  *itMat++ = Smy2xt;
  *itMat++ = Smyxt;  // err
  *itMat++ = Smxyyt;
  *itMat++ = Smy2yt;  // err
  *itMat++ = Smyyt;
  *itMat++ = Sxyxt2pxyyt2;
  *itMat++ = Sy2xt2py2yt2;

  // fill vector
  *itVect++ = Sxxt;
  *itVect++ = Syxt;
  *itVect++ = Sxt;
  *itVect++ = Sxyt;
  *itVect++ = Syyt;
  *itVect++ = Syt;
  *itVect++ = Smxxt2pxyt2;
  *itVect++ = Smyxt2pyyt2;
}

template <typename Vertex, typename F>
void GlobalWarps<Vertex, F>::composeMatAndVectPersp(Vertex *before,
                                                    Vertex *after,
                                                    unsigned vertCnt, F g, F h,
                                                    F *mat, F *vect,
                                                    float weights[]) {
  // allocate space
  int N = 0;
  F w;
  F x, y;
  F xt, yt;
  F Sx2 = 0.0, Sxy = 0.0, Sy2 = 0.0, Sx = 0.0, Sy = 0.0, Smx2xt = 0.0,
    Smxyxt = 0.0, Smxxt = 0, Smx2yt = 0.0, Smxyyt = 0.0, Smxyt = 0.0,
    Sx2xt2px2yt2 = 0.0, Smy2xt = 0.0, Smyxt = 0.0, Smy2yt = 0.0, Smyyt = 0.0,
    Sxyxt2pxyyt2 = 0.0, Sy2xt2py2yt2 = 0;
  F Sxxt = 0.0, Syxt = 0.0, Sxt = 0.0, Sxyt = 0.0, Syyt = 0.0, Syt = 0.0,
    Smxxt2pxyt2 = 0.0, Smyxt2pyyt2 = 0.0;
  F *itMat = mat;
  F *itVect = vect;

  // generate sums
  for (unsigned o = 0; o < vertCnt; o++) {
    x = before[o].x();
    xt = after[o].x();
    y = before[o].y();
    yt = after[o].y();
    w = 1.0 * weights[o] / (g * x + h * y + 1);

    Sx2 += w * x * x;
    Sxy += w * x * y;
    Sy2 += w * y * y;
    Sx += w * x;
    Sy += w * y;
    ++N;
    Smx2xt += -w * x * x * xt;
    Smxyxt += -w * x * y * xt;
    Smxxt += -w * x * xt;
    Smx2yt += -w * x * x * yt;
    Smxyyt += -w * x * y * yt;
    Smxyt += -w * x * yt;
    Sx2xt2px2yt2 += w * x * x * (xt * xt + yt * yt);
    Smy2xt += -w * y * y * xt;
    Smyxt += -w * y * xt;
    Smy2yt += -w * y * y * yt;
    Smyyt += -w * y * yt;
    Sxyxt2pxyyt2 += w * (x * y) * (xt * xt + yt * yt);
    Sy2xt2py2yt2 += w * y * y * (xt * xt + yt * yt);

    Sxxt += w * x * xt;
    Syxt += w * y * xt;
    Sxt += w * xt;
    Sxyt += w * x * yt;
    Syyt += w * y * yt;
    Syt += w * yt;
    Smxxt2pxyt2 += -w * x * (xt * xt + yt * yt);
    Smyxt2pyyt2 += -w * y * (xt * xt + yt * yt);
  }

  // fill matrix
  *itMat++ = Sx2;
  *itMat++ = Sxy;
  *itMat++ = Sy2;
  *itMat++ = Sx;
  *itMat++ = Sy;
  *itMat++ = N;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sx2;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sxy;
  *itMat++ = Sy2;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = 0;
  *itMat++ = Sx;
  *itMat++ = Sy;
  *itMat++ = N;

  *itMat++ = Smx2xt;
  *itMat++ = Smxyxt;
  *itMat++ = Smxxt;
  *itMat++ = Smx2yt;
  *itMat++ = Smxyyt;
  *itMat++ = Smxyt;
  *itMat++ = Sx2xt2px2yt2;

  *itMat++ = Smxyxt;
  *itMat++ = Smy2xt;
  *itMat++ = Smyxt;  // err
  *itMat++ = Smxyyt;
  *itMat++ = Smy2yt;  // err
  *itMat++ = Smyyt;
  *itMat++ = Sxyxt2pxyyt2;
  *itMat++ = Sy2xt2py2yt2;

  // fill vector
  *itVect++ = Sxxt;
  *itVect++ = Syxt;
  *itVect++ = Sxt;
  *itVect++ = Sxyt;
  *itVect++ = Syyt;
  *itVect++ = Syt;
  *itVect++ = Smxxt2pxyt2;
  *itVect++ = Smyxt2pyyt2;
}

template <typename Vertex, typename F>
void GlobalWarps<Vertex, F>::composeMatAndVectAffine(Vertex *before,
                                                     Vertex *after,
                                                     unsigned vertCnt, F *mat,
                                                     F *vect1, F *vect2) {
  // allocate space
  int N = 0;
  F x, y, xt, yt;
  F Sx2 = 0.0, Sxy = 0.0, Sy2 = 0.0, Sx = 0.0, Sy = 0.0, Sxtx = 0.0, Sxty = 0.0,
    Sytx = 0, Syty = 0.0, Sxt = 0.0, Syt = 0.0;

  // generate sums
  for (unsigned o = 0; o < vertCnt; o++) {
    x = before[o].x();
    xt = after[o].x();
    y = before[o].y();
    yt = after[o].y();

    Sx2 += x * x;
    Sy2 += y * y;
    Sxy += x * y;
    Sx += x;
    Sy += y;
    Sxtx += xt * x;
    Sxty += xt * y;
    Sxt += xt;
    Sytx += yt * x;
    Syty += yt * y;
    Syt += yt;
    ++N;
  }

  F m[3][3] = {Sx2, Sxy, Sy2, Sx, Sy, N};
  F v1[3] = {Sxtx, Sxty, Sxt};
  F v2[3] = {Sytx, Syty, Syt};

  memcpy(mat, m, 3 * 3 * sizeof(F));
  memcpy(vect1, v1, 3 * sizeof(F));
  memcpy(vect2, v2, 3 * sizeof(F));
}

template <typename Vertex, typename F>
typename GlobalWarps<Vertex, F>::SolveReturnCode
GlobalWarps<Vertex, F>::solvePersp(F *mat, F *vect) {
  F res[TEST_GYRO_V_STAB_PERSP_DIM];

  if (choleski_solve(mat, vect, res, (uint8_t)TEST_GYRO_V_STAB_PERSP_DIM,
                     (void *)mp_scratch)) {
    // nonzero return code indicates an error
    return failedDetZero;
  } else {
    // check coefficients deltas
    F *itOld = mp_coeffs;
    F *itNew = res;
    std::vector<float>::const_iterator itDeltas = m_deltas.begin();
    bool ready = true;
    for (unsigned i = 0; i < TEST_GYRO_V_STAB_PERSP_DIM; i++) {
      if (abs(*itOld++ - *itNew++) > *itDeltas++) {
        ready = false;
      }
    }
    memcpy(mp_coeffs, res, sizeof(res));
    return ready ? succeededSmallChange : succeededBigChange;
  }
}

template <typename Vertex, typename F>
typename GlobalWarps<Vertex, F>::SolveReturnCode
GlobalWarps<Vertex, F>::solveAffine(F *mat, F *vect1, F *vect2) {
  F res[TEST_GYRO_V_STAB_AFF_DIM];
  if (choleski_solve(mat, vect1, res, (uint8_t)TEST_GYRO_V_STAB_AFF_DIM / 2,
                     (void *)mp_scratch)) {
    // nonzero return code indicates an error
    return failedDetZero;
  } else {
    if (choleski_solve(mat, vect2, (res + 3),
                       (uint8_t)TEST_GYRO_V_STAB_AFF_DIM / 2,
                       (void *)mp_scratch)) {
      // nonzero return code indicates an error
      return failedDetZero;
    } else {
      // got it!
      memcpy(mp_coeffs, res, sizeof(res));
      return succeededSmallChange;
    }
  }
}

}  // namespace GyroVStab
}  // namespace Test
