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
#include <flow/follower/matched_stamp.h>

using namespace flow;
using namespace flow::follower;


struct FollowerMatchedStamp : ::testing::Test, MatchedStamp<Dispatch<int, int>, NoLock>
{
  FollowerMatchedStamp() :
    MatchedStamp<Dispatch<int, int>, NoLock>{}
  {}

  void SetUp() final
  {
    this->reset();
  }
};


TEST_F(FollowerMatchedStamp, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerMatchedStamp, RetryOnDataTooOld)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{0, 0});

  CaptureRange<int> t_range{1, 0};

  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerMatchedStamp, AbortOnDataTooNew)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{1, 0});

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerMatchedStamp, PrimedOnMatchedStamp)
{
  std::vector<Dispatch<int, int>> data;

  this->inject(Dispatch<int, int>{1, 0});

  CaptureRange<int> t_range{1, 1};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}

#endif  // DOXYGEN_SKIP
