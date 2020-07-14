/**
 * @copyright 2020 Fetch Robotics Inc.
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

#endif  // DOXYGEN_SKIP
