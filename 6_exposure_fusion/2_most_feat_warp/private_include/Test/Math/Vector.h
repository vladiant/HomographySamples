/*
 * Vector.h
 *
 *  Created on: Oct 17, 2011
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <sstream>

#include "../CoreCommon.h"
#include "MathCommon.h"

namespace Test {
namespace Math {

template <typename T, unsigned Size>
struct Vector {
  typedef T value_type;
  typedef T& reference;
  typedef T* pointer;
  typedef const T* const_pointer;
  enum { nElements = Size };

  Vector();
  explicit Vector(const value_type& initVal);
  template <typename T2>
  explicit Vector(const Vector<T2, Size>& v);
  Vector(const value_type& x, const value_type& y);
  Vector(const value_type& x, const value_type& y, const value_type& z);
  Vector(const value_type& w, const value_type& x, const value_type& y,
         const value_type& z);
  void initAlElements(const value_type& initVal);
  static unsigned size();
  Vector operator-() const;
  value_type& operator[](unsigned i);
  const value_type& operator[](unsigned i) const;
  value_type& x();
  const value_type& x() const;
  value_type& y();
  const value_type& y() const;
  value_type& z();
  const value_type& z() const;
  bool operator==(const Vector& rhs) const;
  bool operator!=(const Vector& rhs) const;
  bool operator<(const Vector& rhs) const;
  bool operator<=(const Vector& rhs) const;
  Vector& operator+=(const Vector& rhs);
  Vector& operator+=(const value_type& rhs);
  Vector& operator-=(const Vector& rhs);
  Vector& operator-=(const value_type& rhs);
  Vector& operator*=(const value_type& rhs);
  Vector& operator/=(const Vector& rhs);
  Vector& operator/=(const value_type& rhs);
  Vector operator+(const Vector& rhs) const;
  Vector operator+(const value_type& rhs) const;
  Vector operator-(const Vector& rhs) const;
  Vector operator-(const value_type& rhs) const;
  value_type operator*(const Vector& rhs) const;
  Vector operator*(const value_type& rhs) const;
  Vector operator/(const value_type& rhs) const;
  Vector& abs();
  Vector getAbs() const;
  Vector crossProduct(const Vector& rhs) const;
  value_type dotProduct(const Vector& rhs) const;
  value_type dotProduct() const;
  value_type norm() const;
  void normalize();
  Vector normalized() const;
  template <unsigned Length>
  Vector<T, Length>& subVector(unsigned startIndex);
  Vector multiplyElementByElement(const Vector& v) const;
  value_type angle(const Vector& v) const;
  value_type angleCos(const Vector& v) const;
  bool isIntervalInsideInterval(const Vector& v) const;
  void cutOffTop(const Vector& cutOffThreshold);
  void cutOffBottom(const Vector& cutOffThreshold);
  value_type min() const;
  value_type max() const;

 protected:
  T m_Vals[Size];

 private:
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(
      Size | 0);  // This object must have at least 1 dimension
};

template <typename Stream, typename VectorValue, unsigned VectorSize>
Stream& operator<<(Stream& s, IN const Vector<VectorValue, VectorSize>& v);

template <typename Stream, typename VectorValue, unsigned VectorSize>
Stream& operator>>(Stream& s, OUT Vector<VectorValue, VectorSize>& v);

typedef Vector<float, 1> VectorF1;
typedef Vector<float, 2> VectorF2;
typedef Vector<float, 3> VectorF3;
typedef Vector<float, 4> VectorF4;

template <typename Stream, typename VectorValue, unsigned VectorSize>
Stream& operator<<(Stream& s, IN const Vector<VectorValue, VectorSize>& v) {
  s << "{ ";
  for (unsigned i = 0; i < VectorSize - 1; ++i) {
    s << v[i] << ", ";
  }
  s << v[VectorSize - 1] << " }";
  return s;
}

template <typename Stream, typename VectorValue, unsigned VectorSize>
Stream& operator>>(Stream& s, OUT Vector<VectorValue, VectorSize>& v) {
  char c;

  s >> std::skipws;
  s >> c;
  if (c != '{') {
    s.setstate(Stream::failbit);
    return s;
  }

  for (unsigned i = 0; i < VectorSize - 1; ++i) {
    s >> std::skipws >> v[i] >> std::skipws;
    s >> c;
    if (c != ',') {
      s.setstate(Stream::failbit);
      return s;
    }
  }
  s >> std::skipws >> v[VectorSize - 1] >> std::skipws;
  s >> c;
  if (c != '}') {
    s.setstate(Stream::failbit);
    return s;
  }

  return s;
}

template <typename T, unsigned Size>
Vector<T, Size>::Vector() {}

template <typename T, unsigned Size>
Vector<T, Size>::Vector(const value_type& initVal) {
  initAlElements(initVal);
}

template <typename T, unsigned Size>
template <typename T2>
Vector<T, Size>::Vector(const Vector<T2, Size>& v) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] = T(v[i]);
  }
}

template <typename T, unsigned Size>
Vector<T, Size>::Vector(const value_type& x, const value_type& y) {
  m_Vals[0] = x;
  m_Vals[1] = y;
}

template <typename T, unsigned Size>
Vector<T, Size>::Vector(const value_type& x, const value_type& y,
                        const value_type& z) {
  m_Vals[0] = x;
  m_Vals[1] = y;
  m_Vals[2] = z;
}

template <typename T, unsigned Size>
Vector<T, Size>::Vector(const value_type& w, const value_type& x,
                        const value_type& y, const value_type& z) {
  m_Vals[0] = w;
  m_Vals[1] = x;
  m_Vals[2] = y;
  m_Vals[3] = z;
}

template <typename T, unsigned Size>
void Vector<T, Size>::initAlElements(const T& initVal) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] = initVal;
  }
}

template <typename T, unsigned Size>
unsigned Vector<T, Size>::size() {
  return Size;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator-() const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result.m_Vals[i] = -m_Vals[i];
  }
  return result;
}

template <typename T, unsigned Size>
T& Vector<T, Size>::operator[](unsigned i) {
  TEST_ALGORITHMS_CORE_ASSERT(0 <= i && i < Size);
  return m_Vals[i];
}

template <typename T, unsigned Size>
const T& Vector<T, Size>::operator[](unsigned i) const {
  TEST_ALGORITHMS_CORE_ASSERT(0 <= i && i < Size);
  return m_Vals[i];
}

template <typename T, unsigned Size>
T& Vector<T, Size>::x() {
  return m_Vals[0];
}

template <typename T, unsigned Size>
const T& Vector<T, Size>::x() const {
  return m_Vals[0];
}

template <typename T, unsigned Size>
T& Vector<T, Size>::y() {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(Size > 1);
  return m_Vals[1];
}

template <typename T, unsigned Size>
const T& Vector<T, Size>::y() const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(Size > 1);
  return m_Vals[1];
}

template <typename T, unsigned Size>
T& Vector<T, Size>::z() {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(Size > 2);
  return m_Vals[2];
}

template <typename T, unsigned Size>
const T& Vector<T, Size>::z() const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(Size > 2);
  return m_Vals[2];
}

template <typename T, unsigned Size>
bool Vector<T, Size>::operator==(const Vector<T, Size>& rhs) const {
  for (unsigned i = 0; i < Size; ++i) {
    if (m_Vals[i] != rhs.m_Vals[i]) {
      return false;
    }
  }

  return true;
}

template <typename T, unsigned Size>
bool Vector<T, Size>::operator!=(const Vector<T, Size>& rhs) const {
  return !(*this == rhs);
}

template <typename T, unsigned Size>
bool Vector<T, Size>::operator<(const Vector& rhs) const {
  for (unsigned i = 0; i < Size; ++i) {
    if (!(m_Vals[i] < rhs.m_Vals[i])) {
      return false;
    }
  }
  return true;
}

template <typename T, unsigned Size>
bool Vector<T, Size>::operator<=(const Vector& rhs) const {
  for (unsigned i = 0; i < Size; ++i) {
    if (!(m_Vals[i] <= rhs.m_Vals[i])) {
      return false;
    }
  }
  return true;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator+=(const Vector<T, Size>& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] += rhs.m_Vals[i];
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator+=(const value_type& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] += rhs;
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator-=(const Vector<T, Size>& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] -= rhs.m_Vals[i];
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator-=(const value_type& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] -= rhs;
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator*=(const value_type& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] *= rhs;
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator/=(const Vector<T, Size>& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] /= rhs.m_Vals[i];
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::operator/=(const value_type& rhs) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] /= rhs;
  }

  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator+(const Vector<T, Size>& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result.m_Vals[i] = m_Vals[i] + rhs.m_Vals[i];
  }

  return result;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator+(const value_type& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result.m_Vals[i] = m_Vals[i] + rhs;
  }

  return result;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator-(const Vector<T, Size>& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result.m_Vals[i] = m_Vals[i] - rhs.m_Vals[i];
  }

  return result;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator-(const value_type& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result.m_Vals[i] = m_Vals[i] - rhs;
  }

  return result;
}

template <typename T, unsigned Size>
T Vector<T, Size>::operator*(const Vector<T, Size>& rhs) const {
  return dotProduct(rhs);
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::crossProduct(
    const Vector<T, Size>& rhs) const {
  TEST_ALGORITHMS_CORE_STATIC_ASSERT(Size == 3);
  Vector<T, Size> result;
  result[0] = m_Vals[1] * rhs[2] - m_Vals[2] * rhs[1];
  result[1] = m_Vals[3] * rhs[0] - m_Vals[0] * rhs[3];
  result[2] = m_Vals[0] * rhs[1] - m_Vals[1] * rhs[0];

  return result;
}

template <typename T, unsigned Size>
T Vector<T, Size>::dotProduct(const Vector<T, Size>& rhs) const {
  value_type result = value_type(0);
  for (unsigned i = 0; i < Size; ++i) {
    result += m_Vals[i] * rhs.m_Vals[i];
  }

  return result;
}

template <typename T, unsigned Size>
T Vector<T, Size>::dotProduct() const {
  return this->dotProduct(*this);
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator*(const value_type& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result[i] = m_Vals[i] * rhs;
  }

  return result;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::operator/(const value_type& rhs) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result[i] = m_Vals[i] / rhs;
  }

  return result;
}

template <typename T, unsigned Size>
Vector<T, Size>& Vector<T, Size>::abs() {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] = Math::abs(m_Vals[i]);
  }
  return *this;
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::getAbs() const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result[i] = Math::abs(m_Vals[i]);
  }
  return result;
}

template <typename T, unsigned Size>
T Vector<T, Size>::norm() const {
  return sqrt(*this * *this);
}

template <typename T, unsigned Size>
void Vector<T, Size>::normalize() {
  value_type n = norm();
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] /= n;
  }
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::normalized() const {
  value_type n = norm();
  Vector<value_type, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result[i] = m_Vals[i] / n;
  }
  return result;
}

template <typename T, unsigned Size>
template <unsigned Length>
Vector<T, Length>& Vector<T, Size>::subVector(unsigned startIndex) {
  TEST_ALGORITHMS_CORE_ASSERT(0 <= startIndex && startIndex + Length <= Size);
  return *reinterpret_cast<Vector<value_type, Length>*>(&m_Vals[startIndex]);
}

template <typename T, unsigned Size>
Vector<T, Size> Vector<T, Size>::multiplyElementByElement(
    const Vector<T, Size>& v) const {
  Vector<T, Size> result;
  for (unsigned i = 0; i < Size; ++i) {
    result[i] = m_Vals[i] * v[i];
  }
  return result;
}

template <typename T, unsigned Size>
T Vector<T, Size>::angle(const Vector& v) const {
  return acos(angleCos(v));
}

template <typename T, unsigned Size>
T Vector<T, Size>::angleCos(const Vector& v) const {
  return (*this * v) / (sqrt((*this * *this) * (v * v)));
}

template <typename T, unsigned Size>
bool Vector<T, Size>::isIntervalInsideInterval(const Vector<T, Size>& v) const {
  return m_Vals[0] <= v.m_Vals[0] && v.m_Vals[0] <= m_Vals[1] &&
         m_Vals[0] <= v.m_Vals[1] && v.m_Vals[1] <= m_Vals[1];
}

template <typename T, unsigned Size>
void Vector<T, Size>::cutOffTop(const Vector<T, Size>& cutOffThreshold) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] = std::min(m_Vals[i], cutOffThreshold.m_Vals[i]);
  }
}

template <typename T, unsigned Size>
void Vector<T, Size>::cutOffBottom(const Vector<T, Size>& cutOffThreshold) {
  for (unsigned i = 0; i < Size; ++i) {
    m_Vals[i] = std::max(m_Vals[i], cutOffThreshold.m_Vals[i]);
  }
}

template <typename T, unsigned Size>
typename Vector<T, Size>::value_type Vector<T, Size>::min() const {
  value_type result = m_Vals[0];
  for (unsigned i = 1; i < Size; ++i) {
    if (m_Vals[i] < result) {
      result = m_Vals[i];
    }
  }
  return result;
}

template <typename T, unsigned Size>
typename Vector<T, Size>::value_type Vector<T, Size>::max() const {
  value_type result = m_Vals[0];
  for (unsigned i = 1; i < Size; ++i) {
    if (m_Vals[i] > result) {
      result = m_Vals[i];
    }
  }
  return result;
}

}  // namespace Math
}  // namespace Test
