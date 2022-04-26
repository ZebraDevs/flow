/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/captor/nolock.hpp>
#include <flow/follower/closest_before.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerClosestBefore : ::testing::Test, ClosestBefore<Dispatch<int, optional<int>>, NoLock>
{
  static constexpr int PERIOD = 5;
  static constexpr int DELAY = 3;

  std::vector<Dispatch<int, optional<int>>> data;

  FollowerClosestBefore() : ClosestBefore<Dispatch<int, optional<int>>, NoLock>{PERIOD, DELAY} {}

  void SetUp() final
  {
    this->reset();
    data.clear();
  }

  void TearDown() final
  {
    this->inspect([](const Dispatch<int, optional<int>>& element) {
      ASSERT_TRUE(element.value) << "Queue element invalid at stamp(" << element.stamp
                                 << "). Element is nullopt; likely moved erroneously during capture";
    });

    for (const auto& element : data)
    {
      ASSERT_TRUE(element.value) << "Capture element invalid at stamp(" << element.stamp
                                 << "). Element is nullopt; likely moved erroneously during capture";
    }
  }
};
constexpr int FollowerClosestBefore::PERIOD;
constexpr int FollowerClosestBefore::DELAY;


TEST_F(FollowerClosestBefore, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerClosestBefore, CaptureAbortTooNew)
{
  this->inject(Dispatch<int, optional<int>>{1, 1});

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerClosestBefore, CaptureAbortAtDataBoundary)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  std::vector<Dispatch<int, optional<int>>> data;
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
    this->inject(Dispatch<int, optional<int>>{t, t});
  }
  const int t_target = t;

  ASSERT_EQ(this->size(), 2UL * (DELAY + PERIOD));

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(*get_value(data.back()), t_target - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(PERIOD + DELAY + 1));
}


TEST_F(FollowerClosestBefore, CapturePrimedAtDataBoundaryFilledPast)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t, t});
  }
  const int t_target = t + DELAY;

  // Inject messages after t_mid time
  std::size_t M = 2UL * (DELAY + PERIOD);
  while (M--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t, t});
  }

  ASSERT_EQ(this->size(), 4UL * (DELAY + PERIOD));

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(*get_value(data.back()), t_target - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(3UL * (DELAY + PERIOD) - 2));
}


TEST_F(FollowerClosestBefore, CapturePrimedClosestBeforeDataBeforePeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t + PERIOD, t});
  }
  const int t_target = t + PERIOD;

  ASSERT_EQ(this->size(), 10U);

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(*get_value(data.back()), t - DELAY - PERIOD);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(PERIOD + DELAY + 1));
}


TEST_F(FollowerClosestBefore, CaptureAbortClosestBeforeDataAfterPeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t + PERIOD, t});
  }
  const int t_target = 0;

  ASSERT_EQ(this->size(), 10U);

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));

  ASSERT_EQ(this->size(), 10U);
}


TEST_F(FollowerClosestBefore, LocateRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocateAbortTooNew)
{
  this->inject(Dispatch<int, optional<int>>{1, 1});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocateAbortAtDataBoundary)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocatePrimedAtDataBoundary)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t, t});
  }
  const int t_target = t;

  ASSERT_EQ(this->size(), 2UL * (DELAY + PERIOD));

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocatePrimedAtDataBoundaryFilledPast)
{
  int t = 0;

  std::size_t N = 2UL * (DELAY + PERIOD);
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t, t});
  }
  const int t_target = t + DELAY;

  // Inject messages after t_mid time
  std::size_t M = 2UL * (DELAY + PERIOD);
  while (M--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t, t});
  }

  ASSERT_EQ(this->size(), 4UL * (DELAY + PERIOD));

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocatePrimedClosestBeforeDataBeforePeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t + PERIOD, t});
  }
  const int t_target = t + PERIOD;

  ASSERT_EQ(this->size(), 10U);

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, LocateAbortClosestBeforeDataAfterPeriod)
{
  int t = 0;

  std::size_t N = 10;
  while (N--)
  {
    t += 1;
    this->inject(Dispatch<int, optional<int>>{t + PERIOD, t});
  }
  const int t_target = 0;

  ASSERT_EQ(this->size(), 10U);

  CaptureRange<int> t_range{t_target, t_target};
  ASSERT_EQ(State::ABORT, this->locate(t_range));
}


TEST_F(FollowerClosestBefore, RemovalOnAbort)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = 10;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  this->abort(2);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(PERIOD + DELAY + 2));
}

#endif  // DOXYGEN_SKIP
