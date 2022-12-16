/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <iterator>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/captor/nolock.hpp>
#include <flow/follower/any_before.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::follower;


struct FollowerAnyBefore : ::testing::Test, AnyBefore<Dispatch<int, optional<int>>, NoLock>
{
  static constexpr int DELAY = 1;

  std::vector<Dispatch<int, optional<int>>> data;

  FollowerAnyBefore(bool inclusive_boundary) : AnyBefore<Dispatch<int, optional<int>>, NoLock>{DELAY, inclusive_boundary} {}

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
constexpr int FollowerAnyBefore::DELAY;

// AnyBefore Follower with Non-Inclusive Capture Boundary
struct FollowerAnyBeforeNonInclusive : FollowerAnyBefore
{
  FollowerAnyBeforeNonInclusive() : FollowerAnyBefore(false) {}
};

// AnyBefore Follower with Inclusive Capture Boundary
struct FollowerAnyBeforeInclusive : FollowerAnyBefore
{
  FollowerAnyBeforeInclusive() : FollowerAnyBefore(true) {}
};

TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnDataAnyBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedOnDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedMultiDataAnyBeforeBoundary)
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


TEST_F(FollowerAnyBeforeNonInclusive, CapturePrimedMultiDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 0, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});

  ASSERT_EQ(this->size(), 3U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnDataAnyBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedOnDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedMultiDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, LocatePrimedMultiDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 0, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeNonInclusive, RemovalOnAbort)
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

  ASSERT_EQ(this->size(), static_cast<std::size_t>(DELAY + 5));
}

TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 0U);
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(this->size(), 1U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnDataAnyBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 1U);

  ASSERT_EQ(data.size(), 1U);
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedOnDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(this->size(), 2U);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), 2U);
}


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedMultiDataAnyBeforeBoundary)
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


TEST_F(FollowerAnyBeforeInclusive, CapturePrimedMultiDataAnyBeforeAndAtBoundary)
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


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnEmpty)
{
  CaptureRange<int> t_range{0, 0};
  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnDataAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnDataAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnDataAnyBeforeAndAfterBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedOnDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedMultiDataAnyBeforeBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY + 1, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, LocatePrimedMultiDataAnyBeforeAndAtBoundary)
{
  CaptureRange<int> t_range{0, 0};

  this->inject(Dispatch<int, optional<int>>{-DELAY - 0, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 1, 1});
  this->inject(Dispatch<int, optional<int>>{-DELAY - 2, 1});

  ASSERT_EQ(State::PRIMED, this->locate(t_range));
}


TEST_F(FollowerAnyBeforeInclusive, RemovalOnAbort)
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

  ASSERT_EQ(this->size(), static_cast<std::size_t>(DELAY + 5));
}

#endif  // DOXYGEN_SKIP
