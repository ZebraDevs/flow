/**
 * @copyright 2019 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <string>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/dispatch.h>

using namespace flow;



TEST(Dispatch, GetData)
{
  Dispatch<int, std::string> d{2, "test-value"};

  EXPECT_EQ(d.data(), "test-value");
}


TEST(Dispatch, GetStamp)
{
  Dispatch<int, std::string> d{3, "test-value"};

  EXPECT_EQ(d.stamp(), 3);
}


TEST(Dispatch, LessThan)
{
  Dispatch<int, std::string> lhs_data{2, "test-value"};
  Dispatch<int, std::string> rhs_data{3, "test-value"};

  EXPECT_LT(lhs_data, rhs_data);
}


#endif  // DOXYGEN_SKIP
