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
#include <flow/follower/count_before.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerCountBefore : ::testing::Test, CountBefore<Dispatch<int, optional<int>>, NoLock>
{
  static constexpr int COUNT = 3;
  static constexpr int DELAY = 3;

  std::vector<Dispatch<int, optional<int>>> data;

  FollowerCountBefore() : CountBefore<Dispatch<int, optional<int>>, NoLock>{COUNT, DELAY} {}

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
constexpr int FollowerCountBefore::COUNT;
constexpr int FollowerCountBefore::DELAY;


TEST_F(FollowerCountBefore, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerCountBefore, CaptureRetryOnToFewBefore)
{
  std::size_t N = COUNT - 1;

  int t = 0 - (COUNT + DELAY);
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};

  const auto size_before_capture = this->size();
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(size_before_capture, this->size());
}


TEST_F(FollowerCountBefore, CaptureAbortOnToFewBeforeWithDataAfter)
{
  std::size_t N = COUNT;

  int t = 0;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};

  const auto size_before_capture = this->size();
  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(size_before_capture, this->size());
}


TEST_F(FollowerCountBefore, CapturePrimedWithExactBefore)
{
  std::size_t N = COUNT;

  int t = 0 - (COUNT + DELAY);
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  const auto size_before_capture = this->size();
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(size_before_capture, this->size());
  ASSERT_EQ(data.size(), static_cast<std::size_t>(COUNT));
}


TEST_F(FollowerCountBefore, LocateRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(FollowerCountBefore, LocateRetryOnToFewBefore)
{
  std::size_t N = COUNT - 1;

  int t = 0 - (COUNT + DELAY);
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(FollowerCountBefore, LocateAbortOnToFewBeforeWithDataAfter)
{
  std::size_t N = COUNT;

  int t = 0;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::ABORT, this->locate(t_range));
}


TEST_F(FollowerCountBefore, LocatePrimedWithExactBefore)
{
  std::size_t N = COUNT;

  int t = 0 - (COUNT + DELAY);
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerCountBefore, PrimedWithExcessBefore)
{
  constexpr static std::size_t EXCESS = 2;

  std::size_t N = COUNT + EXCESS;

  int t = 0 - static_cast<int>(COUNT + EXCESS + DELAY);
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, t});
    t++;
  }

  std::vector<Dispatch<int, optional<int>>> data;
  CaptureRange<int> t_range{0, 0};
  const auto size_before_capture = this->size();
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(size_before_capture - EXCESS, this->size());
  ASSERT_EQ(data.size(), static_cast<std::size_t>(COUNT));

  for (const auto& dispatch : data)
  {
    const auto offset = get_stamp(dispatch) + DELAY;
    ASSERT_LT(offset, 0);
    ASSERT_GE(offset, -COUNT);
  }
}


TEST_F(FollowerCountBefore, RemovalOnAbort)
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

  ASSERT_EQ(this->size(), static_cast<std::size_t>(10));
}

#endif  // DOXYGEN_SKIP
