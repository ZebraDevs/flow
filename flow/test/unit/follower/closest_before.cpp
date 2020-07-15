// Copyright (C) 2020, Fetch Robotics Inc.
//
// This file is part of Flow.
//
// Flow is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Flow is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Flow.  If not, see <https://www.gnu.org/licenses/>.

#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/follower/closest_before.h>

using namespace flow;
using namespace flow::follower;


struct FollowerClosestBefore : ::testing::Test, ClosestBefore<Dispatch<int, int>, NoLock>
{
  static constexpr int PERIOD = 5;
  static constexpr int DELAY = 3;

  FollowerClosestBefore() : ClosestBefore<Dispatch<int, int>, NoLock>{PERIOD, DELAY} {}

  void SetUp() final { this->reset(); }
};
constexpr int FollowerClosestBefore::PERIOD;
constexpr int FollowerClosestBefore::DELAY;


TEST_F(FollowerClosestBefore, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerClosestBefore, CaptureAbortTooNew)
{
  this->inject(Dispatch<int, int>{1, 1});

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerClosestBefore, CaptureAbortAtDataBoundary)
{
  this->inject(Dispatch<int, int>{0, 0});

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerClosestBefore, CapturePrimedAtDataBoundary)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }
  const int t_target = t;

  ASSERT_EQ(this->size(), 2UL * (DELAY + PERIOD));

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.back().data(), t_target - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(PERIOD + DELAY + 1));
}


TEST_F(FollowerClosestBefore, CapturePrimedAtDataBoundaryFilledPast)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }
  const int t_target = t + DELAY;

  // Inject messages after t_mid time
  std::size_t M = 2UL * (DELAY + PERIOD);
  while (M--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }

  ASSERT_EQ(this->size(), 4UL * (DELAY + PERIOD));

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.back().data(), t_target - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(3UL * (DELAY + PERIOD) - 2));
}


TEST_F(FollowerClosestBefore, CapturePrimedClosestBeforeDataBeforePeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t + PERIOD, t});
  }
  const int t_target = t + PERIOD;

  ASSERT_EQ(this->size(), 10U);

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.back().data(), t - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(PERIOD + DELAY + 1));
}


TEST_F(FollowerClosestBefore, CaptureAbortClosestBeforeDataAfterPeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t + PERIOD, t});
  }
  const int t_target = 0;

  ASSERT_EQ(this->size(), 10U);

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));

  ASSERT_EQ(this->size(), 10U);
}


TEST_F(FollowerClosestBefore, DryCaptureRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCaptureAbortTooNew)
{
  this->inject(Dispatch<int, int>{1, 1});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCaptureAbortAtDataBoundary)
{
  this->inject(Dispatch<int, int>{0, 0});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCapturePrimedAtDataBoundary)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }
  const int t_target = t;

  ASSERT_EQ(this->size(), 2UL * (DELAY + PERIOD));

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCapturePrimedAtDataBoundaryFilledPast)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }
  const int t_target = t + DELAY;

  // Inject messages after t_mid time
  std::size_t M = 2UL * (DELAY + PERIOD);
  while (M--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t, t});
  }

  ASSERT_EQ(this->size(), 4UL * (DELAY + PERIOD));

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCapturePrimedClosestBeforeDataBeforePeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t + PERIOD, t});
  }
  const int t_target = t + PERIOD;

  ASSERT_EQ(this->size(), 10U);

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
}


TEST_F(FollowerClosestBefore, DryCaptureAbortClosestBeforeDataAfterPeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, int>{t + PERIOD, t});
  }
  const int t_target = 0;

  ASSERT_EQ(this->size(), 10U);

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->dry_capture(t_range));
}

#endif  // DOXYGEN_SKIP
