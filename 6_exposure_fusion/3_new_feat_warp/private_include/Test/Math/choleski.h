#pragma once

#include <stdint.h>

//***************************************************************************
//***************************************************************************
//***************************************************************************

//****************************************************************************
// choleski_solve()
// Solves linear system of type:
// b = A * x
//'x' is the unknown vector
//'A' is a positive definite matrix (e.g. regression matrix)
//'b' is a vector of the left sides of the linear equations
//
// matr is lower-left triangle part of A, represented in memory as: a00, a10,
//  a11, a20, a21, a22, ...
// vect is the 'b'
//'x' is written in res
// dimentionality is the number of equations, i.e. the dimentionality of A,b,x
// scratch is working memory of size:
// scratch_size = ( dim^2 + 3*dim ) * sizeof(float_t)
// returns 0 if success, -1 otherwise
//
// !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!!
// the equations in A must be ordered in such a way, the elements in the
// diagonal of A are ordered in accending order from A[00] to A[dim,dim]
// This is necessary to improve calculations accuracy
// !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!! NOTE !!!
// The function may fail (return -1) if the determinant of matr is close ot 0,
// meaning some of the unknowns in x can not be determined
//
//*****************************************************************************
template <typename float_t>
int32_t choleski_solve(float_t* matr, float_t* vect, float_t* res,
                       uint8_t dimentionality, void* scratch);

#include "choleski_impl.h"
