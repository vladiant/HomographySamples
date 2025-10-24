/*
 * CoreCommon.h
 *
 *  Created on: May 8, 2012
 */

#pragma once

#ifndef TEST_ALGORITHMS_CORE_RELEASE
#if defined(RELEASE) || defined(NDEBUG)
#define TEST_ALGORITHMS_CORE_RELEASE
#endif
#if !defined(NDEBUG) || defined(DEBUG)
#define TEST_ALGORITHMS_CORE_DEBUG
#endif
#endif

#define DO_EXPAND(VAL) VAL##1
#define EXPAND(VAL) DO_EXPAND(VAL)

#ifndef OUT
#define OUT
#elif (EXPAND(OUT) != 1)
#error "Error: preprocessor definition 'OUT' must be empty"
#endif

#ifndef IN
#define IN
#elif (EXPAND(IN) != 1)
#error "Error: preprocessor definition 'IN' must be empty"
#endif

#ifdef __cplusplus

#include <cassert>

#ifdef TEST_ALGORITHMS_CORE_DEBUG
#define TEST_ALGORITHMS_CORE_ASSERT(x) assert(x)
#else
#define TEST_ALGORITHMS_CORE_ASSERT(x)
#endif

#ifdef TEST_ALGORITHMS_CORE_USE_BOOST
#include <boost/static_assert.hpp>
#define TEST_ALGORITHMS_CORE_STATIC_ASSERT BOOST_STATIC_ASSERT
#else
#define TEST_ALGORITHMS_CORE_STATIC_ASSERT(x)
#endif

#else  // plain C

#define TEST_ALGORITHMS_CORE_ASSERT(x)
#define TEST_ALGORITHMS_CORE_STATIC_ASSERT(x)

#endif
