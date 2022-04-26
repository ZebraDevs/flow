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
#include <flow/follower/matched_stamp.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerMatchedStamp : ::testing::Test, MatchedStamp<Dispatch<int, optional<int>>, NoLock>
{
  FollowerMatchedStamp() : MatchedStamp<Dispatch<int, optional<int>>, NoLock>{} {}

  std::vector<Dispatch<int, optional<int>>> data;

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


TEST_F(FollowerMatchedStamp, CaptureRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}

TEST_F(FollowerMatchedStamp, CaptureRetryOnDataTooOld)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  CaptureRange<int> t_range{1, 0};

  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerMatchedStamp, CaptureAbortOnDataTooNew)
{
  this->inject(Dispatch<int, optional<int>>{1, 0});

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::ABORT, this->capture(std::back_inserter(data), t_range));
}

TEST_F(FollowerMatchedStamp, CapturePrimedOnExact)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 1UL);
  ASSERT_EQ(this->size(), 1UL);
}

TEST_F(FollowerMatchedStamp, LocateRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(FollowerMatchedStamp, LocateRetryOnDataTooOld)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  CaptureRange<int> t_range{1, 0};

  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(FollowerMatchedStamp, LocateAbortOnDataTooNew)
{
  this->inject(Dispatch<int, optional<int>>{1, 0});

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::ABORT, this->locate(t_range));
}

TEST_F(FollowerMatchedStamp, LocatePrimedOnExact)
{
  this->inject(Dispatch<int, optional<int>>{0, 0});

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerMatchedStamp, PrimedOnMatchedStamp)
{
  this->inject(Dispatch<int, optional<int>>{1, 0});

  CaptureRange<int> t_range{1, 1};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}

TEST_F(FollowerMatchedStamp, PrimedOnMatchedRange)
{
  this->inject(Dispatch<int, optional<int>>{1, 1});
  this->inject(Dispatch<int, optional<int>>{2, 2});
  this->inject(Dispatch<int, optional<int>>{3, 3});
  this->inject(Dispatch<int, optional<int>>{4, 4});

  CaptureRange<int> t_range{2, 3};

  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 2UL);
}


TEST_F(FollowerMatchedStamp, RemovalOnAbort)
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

  ASSERT_EQ(this->size(), static_cast<std::size_t>(8));
}

#endif  // DOXYGEN_SKIP
