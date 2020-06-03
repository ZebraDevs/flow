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
#include <flow/follower/any_before.h>

using namespace flow;
using namespace flow::follower;


struct FollowerAnyBefore : ::testing::Test, AnyBefore<Dispatch<int, int>, NoLock>
{
  static constexpr int DELAY = 1;

  FollowerAnyBefore() :
    AnyBefore<Dispatch<int, int>, NoLock>{DELAY}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr int FollowerAnyBefore::DELAY;


TEST_F(FollowerAnyBefore, PrimedOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerAnyBefore, PrimedOnDataAtBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyBefore, PrimedOnDataAfterBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyBefore, PrimedOnDataAnyBeforeBoundary)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, int>{-DELAY - 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBefore, PrimedOnDataAnyBeforeAndAfterBoundary)
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


TEST_F(FollowerAnyBefore, PrimedOnDataAnyBeforeAndAtBoundary)
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


TEST_F(FollowerAnyBefore, PrimedMultiDataAnyBeforeBoundary)
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


TEST_F(FollowerAnyBefore, PrimedMultiDataAnyBeforeAndAtBoundary)
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


TEST_F(FollowerAnyBefore, PrimedOnInitialLoopBackCapture)
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
