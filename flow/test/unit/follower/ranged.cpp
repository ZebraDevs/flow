/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <iterator>
#include <tuple>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/follower/ranged.h>

using namespace flow;
using namespace flow::follower;


struct FollowerRangedTest : ::testing::TestWithParam<::std::tuple<int>>
{
  using CaptorType = Ranged<Dispatch<int, int>, NoLock>;

  void SetUp() final
  {
    p_delay = std::get<0>(GetParam());
  }

  int p_delay = 0;
};


TEST_P(FollowerRangedTest, RetryOnEmpty)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, AbortOnTooFewBeforeZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay+1, 100);
  captor.inject(-p_delay+2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, AbortOnTooFewBeforeNonZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay+1, 100);
  captor.inject(-p_delay+2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::ABORT, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, RetryOnNoneAfterZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay-2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, RetryOnNoneAfterNonZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay+1, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::RETRY, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureOnZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay+1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 2UL);
}


TEST_P(FollowerRangedTest, CaptureOnZeroRangeWithIntermediate)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay, 100);
  captor.inject(-p_delay+1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 3UL);
}


TEST_P(FollowerRangedTest, CaptureOnNonZeroRange)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay+2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 2UL);
}


TEST_P(FollowerRangedTest, CaptureOnNonZeroRangeWithIntermediate)
{
  CaptorType captor{p_delay};

  std::vector<Dispatch<int, int>> data;

  captor.inject(-p_delay-1, 100);
  captor.inject(-p_delay, 100);
  captor.inject(-p_delay+1, 100);
  captor.inject(-p_delay+2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor.capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 4UL);
}


INSTANTIATE_TEST_SUITE_P(
  FollowerRangedTestSweep,
  FollowerRangedTest,
  testing::Combine(
    testing::ValuesIn(std::vector<int>{0, 1, 2})  // delay
  ));

#endif  // DOXYGEN_SKIP
