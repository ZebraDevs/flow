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
#include <flow/driver/chunk.hpp>
#include <flow/utility/optional.hpp>

using namespace flow;
using namespace flow::driver;


struct DriverChunk : ::testing::Test, Chunk<Dispatch<int, optional<int>>, NoLock>
{
  static constexpr std::size_t CHUNK_SIZE = 10;

  std::vector<Dispatch<int, optional<int>>> data;

  DriverChunk() : Chunk<Dispatch<int, optional<int>>, NoLock>{CHUNK_SIZE} {}

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
constexpr std::size_t DriverChunk::CHUNK_SIZE;


TEST_F(DriverChunk, CaptureRetryOnEmpty)
{

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0U);
}


TEST_F(DriverChunk, CaptureContinueLTChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  ASSERT_EQ(this->size(), CHUNK_SIZE / 2);

  // Start processing

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(data.size(), 0U);
}


TEST_F(DriverChunk, CapturePrimedEQChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }


  // Start processing

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(this->size(), CHUNK_SIZE);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), 0U);

  ASSERT_EQ(data.size(), CHUNK_SIZE);
  EXPECT_EQ(t_range.lower_stamp, static_cast<int>(t0));
  EXPECT_EQ(t_range.upper_stamp, static_cast<int>(t0 + CHUNK_SIZE - 1));
}


TEST_F(DriverChunk, CapturePrimedGTChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE + CHUNK_SIZE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }


  // Start processing

  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(this->size(), CHUNK_SIZE + CHUNK_SIZE / 2);
  ASSERT_EQ(State::PRIMED, this->capture(std::back_inserter(data), t_range));
  ASSERT_EQ(this->size(), CHUNK_SIZE / 2);

  ASSERT_EQ(data.size(), CHUNK_SIZE);
  EXPECT_EQ(t_range.lower_stamp, static_cast<int>(t0));
  EXPECT_EQ(t_range.upper_stamp, static_cast<int>(t0 + CHUNK_SIZE - 1));
}


TEST_F(DriverChunk, LocateRetryOnEmpty)
{
  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(DriverChunk, LocateContinueLTChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  ASSERT_EQ(this->size(), CHUNK_SIZE / 2);

  // Start processing
  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::RETRY, this->locate(t_range));
}


TEST_F(DriverChunk, LocatePrimedEQChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  // Start processing
  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::PRIMED, this->locate(t_range));

  EXPECT_EQ(t_range.lower_stamp, static_cast<int>(t0));
  EXPECT_EQ(t_range.upper_stamp, static_cast<int>(t0 + CHUNK_SIZE - 1));
}


TEST_F(DriverChunk, LocatePrimedGTChunkSize)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE + CHUNK_SIZE / 2;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  // Start processing
  CaptureRange<int> t_range{0, 0};

  ASSERT_EQ(State::PRIMED, this->locate(t_range));

  EXPECT_EQ(t_range.lower_stamp, static_cast<int>(t0));
  EXPECT_EQ(t_range.upper_stamp, static_cast<int>(t0 + CHUNK_SIZE - 1));
}


TEST_F(DriverChunk, RemovalOnAbort)
{
  // Start injecting data
  const int t0 = 0;
  int t = t0;
  int N = CHUNK_SIZE + 1;
  while (N--)
  {
    this->inject(Dispatch<int, optional<int>>{t, 1});
    t += 1;
  }

  this->abort(CHUNK_SIZE);

  ASSERT_EQ(this->size(), 1UL);
}

#endif  // DOXYGEN_SKIP
