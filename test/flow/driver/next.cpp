/**
 * @copyright 2020-present Fetch Robotics Inc.
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
#include <flow/captor/nolock.hpp>
#include <flow/driver/next.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::driver;


struct DriverNext : ::testing::Test, Next<Dispatch<int, optional<int>>, NoLock>
{
  DriverNext() : Next<Dispatch<int, optional<int>>, NoLock>{} {}

  void SetUp() final { this->reset(); }
  void TearDown() final
  {
    this->inspect([](const Dispatch<int, optional<int>>& element) {
      ASSERT_TRUE(element.value) << "At stamp(" << element.stamp
                                 << "). Element is nullopt; likely moved erroneously during capture";
    });
  }
};


TEST_F(DriverNext, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(DriverNext, CapturePrimedWithOldest)
{
  const int t = 1;
  this->inject(Dispatch<int, optional<int>>{t + 0, 1});
  this->inject(Dispatch<int, optional<int>>{t + 1, 2});

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));

  EXPECT_EQ(t_range.lower_stamp, t);
  EXPECT_EQ(t_range.upper_stamp, t);
}


TEST_F(DriverNext, LocateRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(DriverNext, LocatePrimedWithOldest)
{
  const int t = 1;
  this->inject(Dispatch<int, optional<int>>{t + 0, 1});
  this->inject(Dispatch<int, optional<int>>{t + 1, 2});

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));

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
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  this->abort(10);

  ASSERT_EQ(this->size(), 1UL);
}

#endif  // DOXYGEN_SKIP
