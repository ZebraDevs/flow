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
#include <flow/driver/throttled.h>

using namespace flow;
using namespace flow::driver;


struct DriverThrottled : ::testing::Test, Throttled<Dispatch<int, int>, NoLock>
{
  static constexpr int THROTTLE_PERIOD = 4;

  DriverThrottled() :
    Throttled<Dispatch<int, int>, NoLock>{THROTTLE_PERIOD}
  {}

  void SetUp() final
  {
    this->reset();
  }
};
constexpr int DriverThrottled::THROTTLE_PERIOD;


TEST_F(DriverThrottled, RetryOnEmpty)
{
  std::vector<Dispatch<int, int>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(DriverThrottled, RetryUnderLengthPeriod)
{
  const int t = 1;
  for (int offset = 0; offset <= THROTTLE_PERIOD/2; ++offset)
  {
    this->inject(Dispatch<int, int>{t+offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(THROTTLE_PERIOD/2 + 1));

  std::vector<Dispatch<int, int>> data;

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_F(DriverThrottled, NPrimedCapturesIntermediateMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N*THROTTLE_PERIOD; ++offset)
  {
    this->inject(Dispatch<int, int>{t+offset, offset});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N*THROTTLE_PERIOD + 1));

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N*THROTTLE_PERIOD - capture*THROTTLE_PERIOD + 1));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N*THROTTLE_PERIOD - capture*THROTTLE_PERIOD - N + 1));

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


TEST_F(DriverThrottled, NPrimedCapturesExactMessages)
{
  constexpr int N = 4;

  const int t = 1;
  for (int offset = 0; offset <= N; ++offset)
  {
    this->inject(Dispatch<int, int>{t+offset*THROTTLE_PERIOD, offset*THROTTLE_PERIOD});
  }
  ASSERT_EQ(this->size(), static_cast<std::size_t>(N + 1));

  for (int capture = 0; capture < N; ++capture)
  {
    std::vector<Dispatch<int, int>> data;

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture + 1));

    CaptureRange<int> t_range{0, 0};
    ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
    ASSERT_EQ(data.size(), 1UL);

    ASSERT_EQ(this->size(), static_cast<std::size_t>(N - capture));

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

#endif  // DOXYGEN_SKIP
