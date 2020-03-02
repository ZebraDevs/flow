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
#include <flow/follower/count.h>

using namespace flow;
using namespace flow::follower;


struct FollowerCount : ::testing::Test, Count<Dispatch<int, int>, NoLock>
{
  static constexpr int DELAY = 2;
  static constexpr size_type N_BEFORE = 5;
  static constexpr size_type M_AFTER = 3;

  FollowerCount() :
    Count<Dispatch<int, int>, NoLock>{DELAY, N_BEFORE, M_AFTER}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr FollowerCount::size_type FollowerCount::N_BEFORE;
constexpr FollowerCount::size_type FollowerCount::M_AFTER;


TEST_F(FollowerCount, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCount, AbortOnTooFewBefore)
{
  int t = 0;

  // Inject an insufficient number of messages before t_target
  size_type N = N_BEFORE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCount, RetryOnTooFewAfter)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject an insufficient number of messages after t_target
  size_type M = M_AFTER / 2;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCount, ReadyOnExactCounts)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}


TEST_F(FollowerCount, ReadyOnRangedTarget)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = 2 * M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target+3};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER + 3);
}


TEST_F(FollowerCount, ReadyOnExcessCounts)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = 2 * N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = 2 * M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}


TEST_F(FollowerCount, PrimedOnInitialLoopBackCapture)
{
  this->setLoopBackMode(true);

  int t = 0;

  // Inject only data before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};

  // First priming attempt allows capture
  ASSERT_EQ(this->size(), N_BEFORE);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), N_BEFORE);

  data.clear();

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }
  ASSERT_EQ(this->size(), N_BEFORE + M_AFTER);

  // Second attempt result in primed, now with data
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}



struct FollowerCountZeroDelay : ::testing::Test, Count<Dispatch<int, int>, NoLock>
{
  static constexpr int DELAY = 0;
  static constexpr size_type N_BEFORE = 5;
  static constexpr size_type M_AFTER = 3;

  FollowerCountZeroDelay() :
    Count<Dispatch<int, int>, NoLock>{DELAY, N_BEFORE, M_AFTER}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr FollowerCountZeroDelay::size_type FollowerCountZeroDelay::N_BEFORE;
constexpr FollowerCountZeroDelay::size_type FollowerCountZeroDelay::M_AFTER;


TEST_F(FollowerCountZeroDelay, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCountZeroDelay, AbortOnTooFewBefore)
{
  int t = 0;

  // Inject an insufficient number of messages before t_target
  size_type N = N_BEFORE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCountZeroDelay, RetryOnTooFewAfter)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject an insufficient number of messages after t_target
  size_type M = M_AFTER / 2;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCountZeroDelay, ReadyOnExactCounts)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}


TEST_F(FollowerCountZeroDelay, ReadyOnRangedTarget)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = 2 * M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target+3};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER + 3);
}


TEST_F(FollowerCountZeroDelay, ReadyOnExcessCounts)
{
  int t = 0;

  // Inject messages before t_target
  size_type N = 2 * N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  // Inject messages after t_target
  size_type M = 2 * M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}


TEST_F(FollowerCountZeroDelay, PrimedOnInitialLoopBackCapture)
{
  this->setLoopBackMode(true);

  int t = 0;

  // Inject only data before t_target
  size_type N = N_BEFORE;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }

  // Stamp between messages
  const int t_target = t;

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};

  // First priming attempt allows capture
  ASSERT_EQ(this->size(), N_BEFORE);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), N_BEFORE);

  data.clear();

  // Inject messages after t_target
  size_type M = M_AFTER;
  while (M--)
  {
    this->inject(Dispatch<int, int>{t-DELAY, 1});
    t += 1;
  }
  ASSERT_EQ(this->size(), N_BEFORE + M_AFTER);

  // Second attempt result in primed, now with data
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), N_BEFORE + M_AFTER);
}

#endif  // DOXYGEN_SKIP
