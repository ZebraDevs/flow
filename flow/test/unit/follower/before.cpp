/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/follower/before.h>

using namespace flow;
using namespace flow::follower;


struct FollowerBefore : ::testing::Test, Before<Dispatch<int, int>, NoLock>
{
  static constexpr int DELAY = 1;

  FollowerBefore() :
    Before<Dispatch<int, int>, NoLock>{DELAY}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr int FollowerBefore::DELAY;


TEST_F(FollowerBefore, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerBefore, PrimedOnDataAtBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerBefore, PrimedOnDataAfterBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerBefore, RetryOnDataBeforeBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerBefore, PrimedOnDataBeforeAndAfterBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 1, 1});
  this->inject(Dispatch<int, int>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerBefore, PrimedOnDataBeforeAndAtBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 1, 1});
  this->inject(Dispatch<int, int>{-DELAY, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerBefore, PrimedMultiDataBeforeBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 1, 1});
  this->inject(Dispatch<int, int>{-DELAY - 2, 1});
  this->inject(Dispatch<int, int>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 3U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerBefore, PrimedMultiDataBeforeAndAtBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 0, 1});
  this->inject(Dispatch<int, int>{-DELAY - 1, 1});
  this->inject(Dispatch<int, int>{-DELAY - 2, 1});

  ASSERT_EQ(this->size(), 3U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerBefore, PrimedOnInitialLoopBackCapture)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY + 1, 1});
  this->inject(Dispatch<int, int>{-DELAY + 2, 1});

  this->setLoopBackMode(true);

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 2U);

  ASSERT_EQ(data.size(), 0U);
}

#endif  // DOXYGEN_SKIP
