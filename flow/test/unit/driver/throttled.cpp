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
#include <flow/driver/throttled.h>

using namespace flow;
using namespace flow::driver;


struct DriverThrottled : ::testing::Test, Throttled<Dispatch<int, int>, NoLock>
{
  static constexpr int THROTTLE_PERIOD = 4;

  DriverThrottled() : Throttled<Dispatch<int, int>, NoLock>{THROTTLE_PERIOD} {}

  void SetUp() final { this->reset(); }
};
constexpr int DriverThrottled::THROTTLE_PERIOD;


TEST_F(DriverThrottled, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(DriverThrottled, CaptureRetryUnderLengthPeriod)
{
  const int t = 1;
  for (int offset = 0; offset <= THROTTLE_PERIOD / 2; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(THROTTLE_PERIOD / 2 + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  // Next is under throttling period
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 0UL);
  }
}


TEST_F(DriverThrottled, CaptureNPrimedCapturesIntermediateMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N * THROTTLE_PERIOD; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD - capture * THROTTLE_PERIOD));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD - capture * THROTTLE_PERIOD - N));

    EXPECT_EQ(t_range.lower_stamp, t + (capture + 1) * THROTTLE_PERIOD);
    EXPECT_EQ(t_range.upper_stamp, t + (capture + 1) * THROTTLE_PERIOD);
  }

  // Next capture attempt should indicate RETRY signal, as there are not enough messages left
  // to check period between messages again
  {
    std::vector<Dispatch<int, int>> data;
    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 0UL);
  }
}


TEST_F(DriverThrottled, CaptureNPrimedCapturesExactMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset * THROTTLE_PERIOD, offset * THROTTLE_PERIOD});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture - 1));

    EXPECT_EQ(t_range.lower_stamp, t + (capture + 1) * THROTTLE_PERIOD);
    EXPECT_EQ(t_range.upper_stamp, t + (capture + 1) * THROTTLE_PERIOD);
  }

  // Next capture attempt should indicate RETRY signal, as there are not enough messages left
  // to check period between messages again
  {
    std::vector<Dispatch<int, int>> data;
    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 0UL);
  }
}


TEST_F(DriverThrottled, DryCaptureRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
}


TEST_F(DriverThrottled, DryCaptureRetryUnderLengthPeriod)
{
  const int t = 1;
  for (int offset = 0; offset <= THROTTLE_PERIOD / 2; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(THROTTLE_PERIOD / 2 + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  // Next is under throttling period
  {
    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
  }
}


TEST_F(DriverThrottled, DryCaptureNPrimedCapturesIntermediateMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N * THROTTLE_PERIOD; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD - capture * THROTTLE_PERIOD));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N * THROTTLE_PERIOD - capture * THROTTLE_PERIOD - N));

    EXPECT_EQ(t_range.lower_stamp, t + (capture + 1) * THROTTLE_PERIOD);
    EXPECT_EQ(t_range.upper_stamp, t + (capture + 1) * THROTTLE_PERIOD);
  }

  // Next capture attempt should indicate RETRY signal, as there are not enough messages left
  // to check period between messages again
  {
    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
  }
}


TEST_F(DriverThrottled, DryCaptureNPrimedCapturesExactMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N; ++offset)
  {
    this->inject(Dispatch<int, int>{t + offset * THROTTLE_PERIOD, offset * THROTTLE_PERIOD});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N + 1));

  // First capture attempt should be primed
  {
    CaptureRange<int> t_range{0, 0};
    std::vector<Dispatch<int, int>> data;
    ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);
  }

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->dry_capture(t_range));
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture - 1));

    EXPECT_EQ(t_range.lower_stamp, t + (capture + 1) * THROTTLE_PERIOD);
    EXPECT_EQ(t_range.upper_stamp, t + (capture + 1) * THROTTLE_PERIOD);
  }

  // Next capture attempt should indicate RETRY signal, as there are not enough messages left
  // to check period between messages again
  {
    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::RETRY, this->dry_capture(t_range));
  }
}

#endif  // DOXYGEN_SKIP
