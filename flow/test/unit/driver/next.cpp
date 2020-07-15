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
#include <flow/driver/next.h>

using namespace flow;
using namespace flow::driver;


struct DriverNext : ::testing::Test, Next<Dispatch<int, int>, NoLock>
{
  DriverNext() : Next<Dispatch<int, int>, NoLock>{} {}

  void SetUp() final { this->reset(); }
};


TEST_F(DriverNext, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(DriverNext, CapturePrimedWithOldest)
{
  const int t = 1;
  this->inject(Dispatch<int, int>{t + 0, 1});
  this->inject(Dispatch<int, int>{t + 1, 2});

  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));

  EXPECT_EQ(t_range.lower_stamp, t);
  EXPECT_EQ(t_range.upper_stamp, t);
}


TEST_F(DriverNext, DryCaptureRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
}


TEST_F(DriverNext, DryCapturePrimedWithOldest)
{
  const int t = 1;
  this->inject(Dispatch<int, int>{t + 0, 1});
  this->inject(Dispatch<int, int>{t + 1, 2});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));

  EXPECT_EQ(t_range.lower_stamp, t);
  EXPECT_EQ(t_range.upper_stamp, t);
}


TEST_F(DriverNext, RemovalOnAbort)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = 10 + 1;
  while (N--)
  {
    this->inject(Dispatch<int, int>{t, 1});
    t += 1;
  }

  this->abort(10);

  ASSERT_EQ(this->size(), 1UL);
}

#endif  // DOXYGEN_SKIP
