/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/follower/latched.h>

using namespace flow;
using namespace flow::follower;


struct FollowerLatched : ::testing::Test, Latched<Dispatch<int, int>, NoLock>
{
  static constexpr int MIN_PERIOD = 5;

  FollowerLatched() :
    Latched<Dispatch<int, int>, NoLock>{MIN_PERIOD}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr int FollowerLatched::MIN_PERIOD;


TEST_F(FollowerLatched, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_F(FollowerLatched, AbortOnDataTooNew)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 0});

  CaptureRange<int> t_range{MIN_PERIOD-1, MIN_PERIOD-1};

  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_F(FollowerLatched, PrimedOnMinPeriodStamp)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 232});

  CaptureRange<int> t_range{MIN_PERIOD, MIN_PERIOD};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1UL);
  ASSERT_EQ(data.size(), 1UL);
  ASSERT_EQ(data.front().data(), 232);
}


TEST_F(FollowerLatched, PrimedOnBeforeMinPeriodStamp)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 232});

  CaptureRange<int> t_range{MIN_PERIOD+1, MIN_PERIOD+1};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1UL);
  ASSERT_EQ(data.size(), 1UL);
  ASSERT_EQ(data.front().data(), 232);
}


TEST_F(FollowerLatched, PrimedOnBeforeMinPeriodStampTakeNewer)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 232});
  this->inject(Dispatch<int, int>{1, 233});

  CaptureRange<int> t_range{MIN_PERIOD+1, MIN_PERIOD+1};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1UL);
  ASSERT_EQ(data.size(), 1UL);
  ASSERT_EQ(data.front().data(), 233);
}


TEST_F(FollowerLatched, PrimedWithLatchedValue)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 232});

  CaptureRange<int> t_range_priming{MIN_PERIOD, MIN_PERIOD};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range_priming));

  this->inject(Dispatch<int, int>{MIN_PERIOD, 233});

  CaptureRange<int> t_range_successor{MIN_PERIOD+1, MIN_PERIOD+1};

  data.clear();

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range_successor));
  ASSERT_EQ(this->size(), 2UL);
  ASSERT_EQ(data.size(), 1UL);
  ASSERT_EQ(data.front().data(), 232);
}

#endif  // DOXYGEN_SKIP
