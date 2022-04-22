/**
 * @copyright 2022-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <tuple>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/utility/apply.hpp>


using namespace flow;

struct ApplyFunc
{
  void operator()(int _arg0) { arg0 = _arg0; }

  void operator()(int _arg0, int _arg1)
  {
    arg0 = _arg0;
    arg1 = _arg1;
  }


  int arg0, arg1;
};

TEST(ApplyEvery, UnarySingleIteration)
{
  ApplyFunc fn;
  apply_every(fn, std::make_tuple(1));

  ASSERT_EQ(fn.arg0, 1);
}

TEST(ApplyEvery, BinarySingleIteration)
{
  ApplyFunc fn;
  apply_every(fn, std::make_tuple(1), std::make_tuple(2));

  ASSERT_EQ(fn.arg0, 1);
  ASSERT_EQ(fn.arg1, 2);
}

TEST(ApplyEvery, UnaryDoubleIteration)
{
  ApplyFunc fn;
  apply_every(fn, std::make_tuple(1, 2));

  ASSERT_EQ(fn.arg0, 2);
}

TEST(ApplyEvery, BinaryDoubleIteration)
{
  ApplyFunc fn;
  apply_every(fn, std::make_tuple(1, 2), std::make_tuple(3, 4));

  ASSERT_EQ(fn.arg0, 2);
  ASSERT_EQ(fn.arg1, 4);
}

#endif  // DOXYGEN_SKIP