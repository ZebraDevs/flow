/**
 * @copyright 2023 Zebra Technologies
 * @author Somesh Daga
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/captor/nolock.hpp>
#include <flow/follower/any_at_or_before.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerAnyAtOrBefore : ::testing::Test, AnyAtOrBefore<Dispatch<int, optional<int>>, NoLock>
{
  static constexpr int DELAY = 1;

  std::vector<Dispatch<int, optional<int>>> data;

  FollowerAnyAtOrBefore() : AnyAtOrBefore<Dispatch<int, optional<int>>, NoLock>{DELAY} {}

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
constexpr int FollowerAnyAtOrBefore::DELAY;

TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnDataAnyAtOrBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnDataAnyAtOrBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedOnDataAnyAtOrBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedMultiDataAnyAtOrBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 3U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerAnyAtOrBefore, CapturePrimedMultiDataAnyAtOrBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 0, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});

  ASSERT_EQ(this->size(), 3U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 3U);
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnDataAnyAtOrBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnDataAnyAtOrBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedOnDataAnyAtOrBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedMultiDataAnyAtOrBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, LocatePrimedMultiDataAnyAtOrBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 0, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyAtOrBefore, RemovalOnAbort)
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

  this->abort(5);

  ASSERT_EQ(this->size(), static_cast<std::size_t>(DELAY + 4));
}

#endif  // DOXYGEN_SKIP
