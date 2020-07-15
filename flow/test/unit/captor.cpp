/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/captor.h>
#include <flow/driver/next.h>
#include <flow/follower/before.h>


using namespace flow;


TEST(CaptorCheckStampType, DefaultCapacity)
{
  driver::Next<Dispatch<int, int>> captor{};
  ASSERT_EQ(captor.get_capacity(), 0UL);
}


TEST(Captor, InspectCallback)
{
  driver::Next<Dispatch<int, int>> captor{};
  captor.inject(0, 1);

  std::size_t call_count = 0;
  captor.inspect([&call_count](const Dispatch<int, int>& dispatch) {
    ++call_count;
    ASSERT_EQ(dispatch.stamp(), 0);
    ASSERT_EQ(dispatch.data(), 1);
  });

  ASSERT_EQ(call_count, 1UL);
}


TEST(Captor, AvailableStampRangeEmpty)
{
  driver::Next<Dispatch<int, int>> captor{};

  const auto range = captor.get_available_stamp_range();

  ASSERT_FALSE(range);
  ASSERT_EQ(range.lower_stamp, StampTraits<int>::max());
  ASSERT_EQ(range.upper_stamp, StampTraits<int>::min());
}


TEST(Captor, AvailableStampRangeNonEmpty)
{
  driver::Next<Dispatch<int, int>> captor{};
  captor.inject(1, 1);
  captor.inject(10, 1);

  const auto range = captor.get_available_stamp_range();

  ASSERT_TRUE(range);
  ASSERT_EQ(range.lower_stamp, 1);
  ASSERT_EQ(range.upper_stamp, 10);
}


TEST(Captor, RemoveAllOnReset)
{
  driver::Next<Dispatch<int, int>> captor{};
  captor.inject(1, 1);
  captor.inject(10, 1);

  ASSERT_EQ(captor.size(), 2UL);

  captor.reset();

  ASSERT_EQ(captor.size(), 0UL);
}

#endif  // DOXYGEN_SKIP
