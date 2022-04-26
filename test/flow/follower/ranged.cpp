/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <cstdint>
#include <iterator>
#include <tuple>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/captor/nolock.hpp>
#include <flow/follower/ranged.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerRangedTest : ::testing::TestWithParam<::std::tuple<int>>
{
  using CaptorType = Ranged<Dispatch<int, optional<int>>, NoLock>;

  void SetUp() final
  {
    p_delay = std::get<0>(GetParam());
    captor = CaptorType{p_delay};
  }

  void TearDown() final
  {
    captor->inspect([](const Dispatch<int, optional<int>>& element) {
      ASSERT_TRUE(element.value) << "At stamp(" << element.stamp
                                 << "). Element is nullopt; likely moved erroneously during capture";
    });
  }

  optional<CaptorType> captor;

  int p_delay = 0;
};


TEST_P(FollowerRangedTest, CaptureRetryOnEmpty)
{
  std::vector<Dispatch<int, optional<int>>> data;

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureAbortOnTooFewBeforeZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureAbortOnTooFewBeforeNonZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::ABORT, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureRetryOnNoneAfterZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay - 2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureRetryOnNoneAfterNonZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::RETRY, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0UL);
}


TEST_P(FollowerRangedTest, CaptureOnZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 2UL);
}


TEST_P(FollowerRangedTest, CaptureOnZeroRangeWithIntermediate)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 3UL);
}


TEST_P(FollowerRangedTest, CaptureOnNonZeroRange)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 2UL);
}


TEST_P(FollowerRangedTest, CaptureOnNonZeroRangeWithIntermediate)
{
  std::vector<Dispatch<int, optional<int>>> data;

  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay, 100);
  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 4UL);
}


TEST_P(FollowerRangedTest, DryCaptureRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureAbortOnTooFewBeforeZeroRange)
{
  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::ABORT, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureAbortOnTooFewBeforeNonZeroRange)
{
  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::ABORT, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureRetryOnNoneAfterZeroRange)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay - 2, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::RETRY, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureRetryOnNoneAfterNonZeroRange)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::RETRY, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureOnZeroRange)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureOnZeroRangeWithIntermediate)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay, 100);
  captor->inject(-p_delay + 1, 100);

  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureOnNonZeroRange)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, DryCaptureOnNonZeroRangeWithIntermediate)
{
  captor->inject(-p_delay - 1, 100);
  captor->inject(-p_delay, 100);
  captor->inject(-p_delay + 1, 100);
  captor->inject(-p_delay + 2, 100);

  CaptureRange<int> t_range{0, 1};
  ASSERT_EQ(State::PRIMED, captor->locate(t_range));
}


TEST_P(FollowerRangedTest, RemovalOnAbort)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = 10;
  while (N--)
  {
    captor->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  captor->abort(2);

  ASSERT_EQ(captor->size(), static_cast<std::size_t>(10));
}


INSTANTIATE_TEST_SUITE_P(
  FollowerRangedTestSweep,
  FollowerRangedTest,
  testing::Combine(testing::ValuesIn(std::vector<int>{0, 1, 2})  // delay
                   ));

#endif  // DOXYGEN_SKIP
