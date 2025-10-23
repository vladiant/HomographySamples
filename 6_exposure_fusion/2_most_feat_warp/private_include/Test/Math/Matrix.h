/*
 * Matrix.h
 *
 *  Created on: Oct 17, 2011
 */

#pragma once

#include <limits>

#include "../CoreCommon.h"
#include "Vector.h"

namespace Test {
namespace Math {

template <typename T, unsigned nRows, unsigned nCols>
struct Matrix {
  typedef T value_type;
  typedef T& reference;
  typedef T* pointer;
  typedef const T* const_pointer;
  enum {
    RowCount = nRows,
    ColCount = nCols,
  };

  Matrix();
  Matrix(const value_type& initVal);
  static Matrix identity();
  void initAlElements(const value_type& initVal);
  unsigned rowCount() const;
  unsigned colCount() const;
  unsigned size() const;
  Vector<value_type, nCols>& operator[](unsigned i);
  const Vector<value_type, nCols>& operator[](unsigned i) const;
  bool operator==(const Matrix& rhs);
  bool operator!=(const Matrix& rhs);
  Matrix& operator+=(const Matrix& rhs);
  Matrix& operator+=(const value_type& rhs);
  Matrix& operator-=(const Matrix& rhs);
  Matrix& operator-=(const value_type& rhs);
  Matrix& operator*=(const value_type& rhs);
  Matrix& operator/=(const value_type& rhs);
  Matrix operator+(const Matrix& rhs) const;
  Matrix operator+(const value_type& rhs) const;
  Matrix operator-(const Matrix& rhs) const;
  Matrix operator-(const value_type& rhs) const;
  Matrix operator*(const value_type& rhs) const;
  Vector<value_type, nRows> operator*(
      const Vector<value_type, nCols>& rhs) const;

  template <unsigned nColsInRightMatrix>
  Matrix<value_type, nCols, nColsInRightMatrix> operator*(
      const Matrix<value_type, nRows, nColsInRightMatrix>& rhs) const;
  value_type trace() const;
  Matrix<value_type, nCols, nRows> transposed() const;
  void transpose();
  value_type determinant() const;
  bool inverse(OUT Matrix& m) const;
  bool inverseAssumeDiagonalForm(OUT Matrix& m) const;
  Matrix inverseAssumeDiagonalForm() const;
  Matrix& invertAssumeDiagonalForm();
  value_type dotProduct(unsigned colIndex,
                        const Vector<value_type, nRows>& v) const;
  void transposeAndMultiplyOnTheRightByDiagonalMatrix(
      const Vector<value_type, nRows>& diagonalMatrix);
  void multiplyOnTheLeftByDiagonalMatrix(
      const Vector<value_type, nRows>& diagonalMatrix);
  Vector<value_type, nRows> multiplyAssumeDiagonalForm(
      const Vector<value_type, nCols>& rhs) const;
  Vector<value_type, nCols> transposeAndMultiply(
      const Vector<value_type, nRows>& rhs) const;
  void setRotationPart(const Vector<value_type, 3>& unitRotationAxis,
                       const value_type& angleSize);

 protected:
  Vector<value_type, nCols> m_Vals[nRows];

 private:
  // This object must have at least 1 row and 1 column
  TEST_ALGORITHMS_CORE_STATIC_ASSERT((nRows | 0) && (nCols | 0));
};

typedef Matrix<float, 1, 1> MatrixF1x1;
typedef Matrix<float, 2, 2> MatrixF2x2;
typedef Matrix<float, 3, 3> MatrixF3x3;
typedef Matrix<float, 4, 4> MatrixF4x4;

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>::Matrix() {}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>::Matrix(const value_type& initVal) {
  initAlElements(initVal);
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::identity() {
  TEST_ALGORITHMS_CORE_ASSERT(nRows == nCols);

  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      result[i][j] = value_type(0);
    }
  }

  for (unsigned i = 0; i < nRows; ++i) {
    result[i][i] = value_type(1);
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
void Matrix<T, nRows, nCols>::initAlElements(const value_type& initVal) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i].initAlElements(initVal);
  }
}

template <typename T, unsigned nRows, unsigned nCols>
unsigned Matrix<T, nRows, nCols>::rowCount() const {
  return nRows;
}

template <typename T, unsigned nRows, unsigned nCols>
unsigned Matrix<T, nRows, nCols>::colCount() const {
  return nCols;
}

template <typename T, unsigned nRows, unsigned nCols>
unsigned Matrix<T, nRows, nCols>::size() const {
  return nRows * nCols;
}

template <typename T, unsigned nRows, unsigned nCols>
Vector<T, nCols>& Matrix<T, nRows, nCols>::operator[](unsigned i) {
  return m_Vals[i];
}

template <typename T, unsigned nRows, unsigned nCols>
const Vector<T, nCols>& Matrix<T, nRows, nCols>::operator[](unsigned i) const {
  return m_Vals[i];
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator+=(const T& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] += rhs;
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
bool Matrix<T, nRows, nCols>::operator==(const Matrix& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      if (m_Vals[i][j] != rhs[i][j]) {
        return false;
      }
    }
  }

  return true;
}

template <typename T, unsigned nRows, unsigned nCols>
bool Matrix<T, nRows, nCols>::operator!=(const Matrix& rhs) {
  return !(*this == rhs);
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator+=(
    const Matrix<T, nRows, nCols>& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] += rhs.m_Vals[i];
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator-=(
    const Matrix<T, nRows, nCols>& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] -= rhs.m_Vals[i];
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator-=(const T& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] -= rhs;
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator*=(const T& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] *= rhs;
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::operator/=(const T& rhs) {
  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i] /= rhs;
  }

  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::operator+(
    const Matrix<T, nRows, nCols>& rhs) const {
  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result.m_Vals[i] = m_Vals[i] + rhs.m_Vals[i];
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::operator+(const T& rhs) const {
  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result.m_Vals[i] = m_Vals[i] + rhs;
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::operator-(
    const Matrix<T, nRows, nCols>& rhs) const {
  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result.m_Vals[i] = m_Vals[i] - rhs.m_Vals[i];
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::operator-(const T& rhs) const {
  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result.m_Vals[i] = m_Vals[i] - rhs;
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::operator*(const T& rhs) const {
  Matrix<T, nRows, nCols> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result.m_Vals[i] = m_Vals[i] * rhs;
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Vector<T, nRows> Matrix<T, nRows, nCols>::operator*(
    const Vector<T, nCols>& rhs) const {
  Vector<T, nRows> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result[i] = m_Vals[i] * rhs;
  }
  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
template <unsigned nColsInRightMatrix>
Matrix<T, nCols, nColsInRightMatrix> Matrix<T, nRows, nCols>::operator*(
    const Matrix<T, nRows, nColsInRightMatrix>& rhs) const {
  Matrix<T, nCols, nColsInRightMatrix> result;
  for (unsigned i = 0; i < nColsInRightMatrix; ++i) {
    for (unsigned j = 0; j < nRows; ++j) {
      result[j][i] = rhs.dotProduct(i, m_Vals[j]);
    }
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
T Matrix<T, nRows, nCols>::trace() const {
  TEST_ALGORITHMS_CORE_ASSERT(nRows == nCols);

  value_type result = value_type(0);
  for (unsigned i = 0; i < nRows; ++i) {
    result += m_Vals[i][i];
  }
  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nCols, nRows> Matrix<T, nRows, nCols>::transposed() const {
  Matrix<T, nCols, nRows> result;
  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      result[j][i] = m_Vals[i][j];
    }
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
void Matrix<T, nRows, nCols>::transpose() {
  TEST_ALGORITHMS_CORE_ASSERT(nRows == nCols);

  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = i + 1; j < nCols; ++j) {
      value_type tmp = m_Vals[j][i];
      m_Vals[j][i] = m_Vals[i][j];
      m_Vals[i][j] = tmp;
    }
  }
}

template <typename T, unsigned nRows, unsigned nCols>
T Matrix<T, nRows, nCols>::determinant() const {
  // 2x2 sub-determinants
  value_type det2_01_01 =
      m_Vals[0][0] * m_Vals[1][1] - m_Vals[0][1] * m_Vals[1][0];
  value_type det2_01_02 =
      m_Vals[0][0] * m_Vals[1][2] - m_Vals[0][2] * m_Vals[1][0];
  value_type det2_01_03 =
      m_Vals[0][0] * m_Vals[1][3] - m_Vals[0][3] * m_Vals[1][0];
  value_type det2_01_12 =
      m_Vals[0][1] * m_Vals[1][2] - m_Vals[0][2] * m_Vals[1][1];
  value_type det2_01_13 =
      m_Vals[0][1] * m_Vals[1][3] - m_Vals[0][3] * m_Vals[1][1];
  value_type det2_01_23 =
      m_Vals[0][2] * m_Vals[1][3] - m_Vals[0][3] * m_Vals[1][2];

  // 3x3 sub-determinants
  value_type det3_201_012 = m_Vals[2][0] * det2_01_12 -
                            m_Vals[2][1] * det2_01_02 +
                            m_Vals[2][2] * det2_01_01;
  value_type det3_201_013 = m_Vals[2][0] * det2_01_13 -
                            m_Vals[2][1] * det2_01_03 +
                            m_Vals[2][3] * det2_01_01;
  value_type det3_201_023 = m_Vals[2][0] * det2_01_23 -
                            m_Vals[2][2] * det2_01_03 +
                            m_Vals[2][3] * det2_01_02;
  value_type det3_201_123 = m_Vals[2][1] * det2_01_23 -
                            m_Vals[2][2] * det2_01_13 +
                            m_Vals[2][3] * det2_01_12;

  return (-det3_201_123 * m_Vals[3][0] + det3_201_023 * m_Vals[3][1] -
          det3_201_013 * m_Vals[3][2] + det3_201_012 * m_Vals[3][3]);
}

template <typename T, unsigned nRows, unsigned nCols>
bool Matrix<T, nRows, nCols>::inverse(OUT Matrix<T, nRows, nCols>& mat) const {
  TEST_ALGORITHMS_CORE_ASSERT(nRows == nCols);
  TEST_ALGORITHMS_CORE_ASSERT(nRows == 4);
  value_type det, invDet;

  switch (nRows) {
    case 4:
      det = determinant();

      if (equal(det, value_type(0))) {
        // Singular matrix
        return false;
      }

      invDet = 1 / det;

      mat[0][0] = invDet * ((-(*this)[1][3] * (*this)[2][2] * (*this)[3][1]) +
                            ((*this)[1][2] * (*this)[2][3] * (*this)[3][1]) +
                            ((*this)[1][3] * (*this)[2][1] * (*this)[3][2]) -
                            ((*this)[1][1] * (*this)[2][3] * (*this)[3][2]) -
                            ((*this)[1][2] * (*this)[2][1] * (*this)[3][3]) +
                            ((*this)[1][1] * (*this)[2][2] * (*this)[3][3]));
      mat[0][1] = invDet * (((*this)[0][3] * (*this)[2][2] * (*this)[3][1]) -
                            ((*this)[0][2] * (*this)[2][3] * (*this)[3][1]) -
                            ((*this)[0][3] * (*this)[2][1] * (*this)[3][2]) +
                            ((*this)[0][1] * (*this)[2][3] * (*this)[3][2]) +
                            ((*this)[0][2] * (*this)[2][1] * (*this)[3][3]) -
                            ((*this)[0][1] * (*this)[2][2] * (*this)[3][3]));
      mat[0][2] = invDet * ((-(*this)[0][3] * (*this)[1][2] * (*this)[3][1]) +
                            ((*this)[0][2] * (*this)[1][3] * (*this)[3][1]) +
                            ((*this)[0][3] * (*this)[1][1] * (*this)[3][2]) -
                            ((*this)[0][1] * (*this)[1][3] * (*this)[3][2]) -
                            ((*this)[0][2] * (*this)[1][1] * (*this)[3][3]) +
                            ((*this)[0][1] * (*this)[1][2] * (*this)[3][3]));
      mat[0][3] = invDet * (((*this)[0][3] * (*this)[1][2] * (*this)[2][1]) -
                            ((*this)[0][2] * (*this)[1][3] * (*this)[2][1]) -
                            ((*this)[0][3] * (*this)[1][1] * (*this)[2][2]) +
                            ((*this)[0][1] * (*this)[1][3] * (*this)[2][2]) +
                            ((*this)[0][2] * (*this)[1][1] * (*this)[2][3]) -
                            ((*this)[0][1] * (*this)[1][2] * (*this)[2][3]));

      mat[1][0] = invDet * (((*this)[1][3] * (*this)[2][2] * (*this)[3][0]) -
                            ((*this)[1][2] * (*this)[2][3] * (*this)[3][0]) -
                            ((*this)[1][3] * (*this)[2][0] * (*this)[3][2]) +
                            ((*this)[1][0] * (*this)[2][3] * (*this)[3][2]) +
                            ((*this)[1][2] * (*this)[2][0] * (*this)[3][3]) -
                            ((*this)[1][0] * (*this)[2][2] * (*this)[3][3]));
      mat[1][1] = invDet * ((-(*this)[0][3] * (*this)[2][2] * (*this)[3][0]) +
                            ((*this)[0][2] * (*this)[2][3] * (*this)[3][0]) +
                            ((*this)[0][3] * (*this)[2][0] * (*this)[3][2]) -
                            ((*this)[0][0] * (*this)[2][3] * (*this)[3][2]) -
                            ((*this)[0][2] * (*this)[2][0] * (*this)[3][3]) +
                            ((*this)[0][0] * (*this)[2][2] * (*this)[3][3]));
      mat[1][2] = invDet * (((*this)[0][3] * (*this)[1][2] * (*this)[3][0]) -
                            ((*this)[0][2] * (*this)[1][3] * (*this)[3][0]) -
                            ((*this)[0][3] * (*this)[1][0] * (*this)[3][2]) +
                            ((*this)[0][0] * (*this)[1][3] * (*this)[3][2]) +
                            ((*this)[0][2] * (*this)[1][0] * (*this)[3][3]) -
                            ((*this)[0][0] * (*this)[1][2] * (*this)[3][3]));
      mat[1][3] = invDet * ((-(*this)[0][3] * (*this)[1][2] * (*this)[2][0]) +
                            ((*this)[0][2] * (*this)[1][3] * (*this)[2][0]) +
                            ((*this)[0][3] * (*this)[1][0] * (*this)[2][2]) -
                            ((*this)[0][0] * (*this)[1][3] * (*this)[2][2]) -
                            ((*this)[0][2] * (*this)[1][0] * (*this)[2][3]) +
                            ((*this)[0][0] * (*this)[1][2] * (*this)[2][3]));

      mat[2][0] = invDet * ((-(*this)[1][3] * (*this)[2][1] * (*this)[3][0]) +
                            ((*this)[1][1] * (*this)[2][3] * (*this)[3][0]) +
                            ((*this)[1][3] * (*this)[2][0] * (*this)[3][1]) -
                            ((*this)[1][0] * (*this)[2][3] * (*this)[3][1]) -
                            ((*this)[1][1] * (*this)[2][0] * (*this)[3][3]) +
                            ((*this)[1][0] * (*this)[2][1] * (*this)[3][3]));
      mat[2][1] = invDet * (((*this)[0][3] * (*this)[2][1] * (*this)[3][0]) -
                            ((*this)[0][1] * (*this)[2][3] * (*this)[3][0]) -
                            ((*this)[0][3] * (*this)[2][0] * (*this)[3][1]) +
                            ((*this)[0][0] * (*this)[2][3] * (*this)[3][1]) +
                            ((*this)[0][1] * (*this)[2][0] * (*this)[3][3]) -
                            ((*this)[0][0] * (*this)[2][1] * (*this)[3][3]));
      mat[2][2] = invDet * ((-(*this)[0][3] * (*this)[1][1] * (*this)[3][0]) +
                            ((*this)[0][1] * (*this)[1][3] * (*this)[3][0]) +
                            ((*this)[0][3] * (*this)[1][0] * (*this)[3][1]) -
                            ((*this)[0][0] * (*this)[1][3] * (*this)[3][1]) -
                            ((*this)[0][1] * (*this)[1][0] * (*this)[3][3]) +
                            ((*this)[0][0] * (*this)[1][1] * (*this)[3][3]));
      mat[2][3] = invDet * (((*this)[0][3] * (*this)[1][1] * (*this)[2][0]) -
                            ((*this)[0][1] * (*this)[1][3] * (*this)[2][0]) -
                            ((*this)[0][3] * (*this)[1][0] * (*this)[2][1]) +
                            ((*this)[0][0] * (*this)[1][3] * (*this)[2][1]) +
                            ((*this)[0][1] * (*this)[1][0] * (*this)[2][3]) -
                            ((*this)[0][0] * (*this)[1][1] * (*this)[2][3]));

      mat[3][0] = invDet * (((*this)[1][2] * (*this)[2][1] * (*this)[3][0]) -
                            ((*this)[1][1] * (*this)[2][2] * (*this)[3][0]) -
                            ((*this)[1][2] * (*this)[2][0] * (*this)[3][1]) +
                            ((*this)[1][0] * (*this)[2][2] * (*this)[3][1]) +
                            ((*this)[1][1] * (*this)[2][0] * (*this)[3][2]) -
                            ((*this)[1][0] * (*this)[2][1] * (*this)[3][2]));
      mat[3][1] = invDet * ((-(*this)[0][2] * (*this)[2][1] * (*this)[3][0]) +
                            ((*this)[0][1] * (*this)[2][2] * (*this)[3][0]) +
                            ((*this)[0][2] * (*this)[2][0] * (*this)[3][1]) -
                            ((*this)[0][0] * (*this)[2][2] * (*this)[3][1]) -
                            ((*this)[0][1] * (*this)[2][0] * (*this)[3][2]) +
                            ((*this)[0][0] * (*this)[2][1] * (*this)[3][2]));
      mat[3][2] = invDet * (((*this)[0][2] * (*this)[1][1] * (*this)[3][0]) -
                            ((*this)[0][1] * (*this)[1][2] * (*this)[3][0]) -
                            ((*this)[0][2] * (*this)[1][0] * (*this)[3][1]) +
                            ((*this)[0][0] * (*this)[1][2] * (*this)[3][1]) +
                            ((*this)[0][1] * (*this)[1][0] * (*this)[3][2]) -
                            ((*this)[0][0] * (*this)[1][1] * (*this)[3][2]));
      mat[3][3] = invDet * ((-(*this)[0][2] * (*this)[1][1] * (*this)[2][0]) +
                            ((*this)[0][1] * (*this)[1][2] * (*this)[2][0]) +
                            ((*this)[0][2] * (*this)[1][0] * (*this)[2][1]) -
                            ((*this)[0][0] * (*this)[1][2] * (*this)[2][1]) -
                            ((*this)[0][1] * (*this)[1][0] * (*this)[2][2]) +
                            ((*this)[0][0] * (*this)[1][1] * (*this)[2][2]));

      return true;
      break;
  }

  return true;
}

template <typename T, unsigned nRows, unsigned nCols>
bool Matrix<T, nRows, nCols>::inverseAssumeDiagonalForm(OUT Matrix& m) const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

  for (unsigned i = 0; i < nRows; ++i) {
    if (equal(m_Vals[i][i], value_type(0))) {
      return false;
    }

    for (unsigned j = 0; j < nCols; ++j) {
      TEST_ALGORITHMS_CORE_ASSERT(equal(m_Vals[i][j], value_type(0)));
      m[i][j] = 0;
    }

    m[i][i] = 1 / m_Vals[i][i];
  }

  return true;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols> Matrix<T, nRows, nCols>::inverseAssumeDiagonalForm()
    const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      TEST_ALGORITHMS_CORE_ASSERT(equal(m_Vals[i][j], value_type(0)));
    }
  }

  Matrix<T, nRows, nCols> result;
  result.initAlElements(value_type(0));
  for (unsigned i = 0; i < nRows; ++i) {
    result[i][i] = 1 / m_Vals[i][i];
  }
  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Matrix<T, nRows, nCols>& Matrix<T, nRows, nCols>::invertAssumeDiagonalForm() {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

#ifdef TEST_GYRO_V_STAB_DEBUG
  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      TEST_ALGORITHMS_CORE_ASSERT(equal(m_Vals[i][j], value_type(0)));
    }
  }
#endif

  for (unsigned i = 0; i < nRows; ++i) {
    m_Vals[i][i] = 1 / m_Vals[i][i];
  }
  return *this;
}

template <typename T, unsigned nRows, unsigned nCols>
T Matrix<T, nRows, nCols>::dotProduct(unsigned colIndex,
                                      const Vector<T, nRows>& v) const {
  value_type result = value_type(0);
  for (unsigned i = 0; i < nRows; ++i) {
    result += m_Vals[i][colIndex] * v[i];
  }

  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
void Matrix<T, nRows, nCols>::transposeAndMultiplyOnTheRightByDiagonalMatrix(
    const Vector<T, nRows>& diagonalMatrix) {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

  this->transpose();
  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      m_Vals[i][j] = m_Vals[i][j] * diagonalMatrix[j];
    }
  }
}

template <typename T, unsigned nRows, unsigned nCols>
void Matrix<T, nRows, nCols>::multiplyOnTheLeftByDiagonalMatrix(
    const Vector<T, nRows>& diagonalMatrix) {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

  for (unsigned i = 0; i < nRows; ++i) {
    for (unsigned j = 0; j < nCols; ++j) {
      m_Vals[i][j] *= diagonalMatrix[i];
    }
  }
}

template <typename T, unsigned nRows, unsigned nCols>
Vector<T, nRows> Matrix<T, nRows, nCols>::multiplyAssumeDiagonalForm(
    const Vector<T, nCols>& rhs) const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(nRows == nCols);

  Vector<T, nRows> result;
  for (unsigned i = 0; i < nRows; ++i) {
    result[i] = m_Vals[i][i] * rhs[i];
  }
  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
Vector<T, nCols> Matrix<T, nRows, nCols>::transposeAndMultiply(
    const Vector<T, nRows>& rhs) const {
  Vector<T, nCols> result;
  for (unsigned i = 0; i < nCols; ++i) {
    result[i] = dotProduct(i, rhs);
  }
  return result;
}

template <typename T, unsigned nRows, unsigned nCols>
void Matrix<T, nRows, nCols>::setRotationPart(
    const Vector<value_type, 3>& unitRotationAxis,
    const value_type& angleSize) {
  TEST_ALGORITHMS_CORE_ASSERT(nRows > 2 && nCols > 2 &&
                              equal(unitRotationAxis.norm(), T(1)));

  value_type cos = cos(angleSize);
  value_type oneMcos = 1 - cos(angleSize);
  value_type sin = sin(angleSize);
  value_type xXoneMcos = unitRotationAxis.x() * oneMcos;
  value_type xx = unitRotationAxis.x() * xXoneMcos;
  value_type xy = unitRotationAxis.y() * xXoneMcos;
  value_type xz = unitRotationAxis.z() * xXoneMcos;
  value_type yXoneMcos = unitRotationAxis.y() * oneMcos;
  value_type yy = unitRotationAxis.y() * yXoneMcos;
  value_type yz = unitRotationAxis.z() * yXoneMcos;
  value_type zz = unitRotationAxis.z() * unitRotationAxis.z() * oneMcos;
  value_type xXsin = unitRotationAxis.x() * sin;
  value_type yXsin = unitRotationAxis.y() * sin;
  value_type zXsin = unitRotationAxis.z() * sin;
  m_Vals[0][0] = cos + xx;
  m_Vals[0][1] = xy - zXsin;
  m_Vals[0][2] = xz + yXsin;
  m_Vals[1][0] = xy + zXsin;
  m_Vals[1][1] = cos + yy;
  m_Vals[1][2] = yz - xXsin;
  m_Vals[2][0] = xz - yXsin;
  m_Vals[2][1] = yz + xXsin;
  m_Vals[2][2] = cos + zz;
}

}  // namespace Math
}  // namespace Test
