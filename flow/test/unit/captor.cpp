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


TEST(CaptorCheckStampType, SingleCaptor)
{
  using C1 = driver::Next<Dispatch<int, int>>;

  detail::check_stamp_type<C1>();
}


TEST(CaptorCheckStampType, MultiCaptor)
{
  using C1 = driver::Next<Dispatch<int, int>>;
  using C2 = follower::Before<Dispatch<int, int>>;

  detail::check_stamp_type<C1, C2>();
}


TEST(CaptorCheckStampType, DefaultCapacity)
{
  driver::Next<Dispatch<int, int>> captor{};
  ASSERT_EQ(captor.get_capacity(), 0UL);
}


TEST(CaptorCheckStampType, InspectCallback)
{
  driver::Next<Dispatch<int, int>> captor{};
  captor.inject(0, 1);

  std::size_t call_count = 0;
  captor.inspect([&call_count](const Dispatch<int, int>& dispatch)
  {
    ++call_count;
    ASSERT_EQ(dispatch.stamp(), 0);
    ASSERT_EQ(dispatch.data(), 1);
  });

  ASSERT_EQ(call_count, 1UL);
}

#endif  // DOXYGEN_SKIP
