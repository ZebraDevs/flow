/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <deque>
#include <iterator>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/drivers.h>
#include <flow/followers.h>
#include <flow/synchronizer.h>

using namespace flow;

/// Tests the Synchronizer class
class SynchronizerTestSuite : public ::testing::Test
{
public:
  using Driver = driver::Next<Dispatch<int, int>, NoLock>;
  using Follower1 = follower::ClosestBefore<Dispatch<int, double>, NoLock>;
  using Follower2 = follower::Before<Dispatch<int, std::string>, NoLock>;

  /// Persistent capture buffer resources
  std::shared_ptr<Driver> driver;
  std::shared_ptr<Follower1> follower1;
  std::shared_ptr<Follower2> follower2;

  /// Constructor
  SynchronizerTestSuite() {}

  /**
   * @brief Test initialization
   */
  void SetUp() final
  {
    driver = std::make_shared<Driver>();
    follower1 = std::make_shared<Follower1>(5, 0);
    follower2 = std::make_shared<Follower2>(0);
  }

  /**
   * @brief Test cleanup
   */
  void TearDown() final {}
};


TEST_F(SynchronizerTestSuite, Reset) { Synchronizer::reset(std::forward_as_tuple(*driver, *follower1, *follower2)); }


TEST_F(SynchronizerTestSuite, CaptureCannotPrimeRetry)
{
  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = Synchronizer::capture(
    std::forward_as_tuple(*driver, *follower1, *follower2),
    std::forward_as_tuple(
      std::back_inserter(driver_output_data),
      std::back_inserter(follower1_output_data),
      std::back_inserter(follower2_output_data)),
    0);

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::RETRY);
}


TEST_F(SynchronizerTestSuite, CaptureCannotPrimeAbort)
{
  driver->inject(Dispatch<int, int>{1, 1});
  follower1->inject(Dispatch<int, double>{7, 2.0});
  follower2->inject(Dispatch<int, std::string>{0, "ok"});

  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = Synchronizer::capture(
    std::forward_as_tuple(*driver, *follower1, *follower2),
    std::forward_as_tuple(
      std::back_inserter(driver_output_data),
      std::back_inserter(follower1_output_data),
      std::back_inserter(follower2_output_data)),
    0);

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::ABORT);
}


TEST_F(SynchronizerTestSuite, CaptureCanPrime)
{
  driver->inject(Dispatch<int, int>{10, 10});
  follower1->inject(Dispatch<int, double>{0, 2.0});
  follower1->inject(Dispatch<int, double>{9, 2.0});
  follower2->inject(Dispatch<int, std::string>{20, "ok"});

  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = Synchronizer::capture(
    std::forward_as_tuple(*driver, *follower1, *follower2),
    std::forward_as_tuple(
      std::back_inserter(driver_output_data),
      std::back_inserter(follower1_output_data),
      std::back_inserter(follower2_output_data)),
    0);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.state, State::PRIMED);

  ASSERT_FALSE(driver_output_data.empty());
  ASSERT_FALSE(follower1_output_data.empty());
  ASSERT_TRUE(follower2_output_data.empty());
}


TEST_F(SynchronizerTestSuite, DryCaptureCannotPrimeRetry)
{
  const auto result = Synchronizer::dry_capture(std::forward_as_tuple(*driver, *follower1, *follower2), 0);

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::RETRY);
}


TEST_F(SynchronizerTestSuite, DryCaptureCannotPrimeAbort)
{
  driver->inject(Dispatch<int, int>{1, 1});
  follower1->inject(Dispatch<int, double>{7, 2.0});
  follower2->inject(Dispatch<int, std::string>{0, "ok"});

  const auto result = Synchronizer::dry_capture(std::forward_as_tuple(*driver, *follower1, *follower2), 0);

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::ABORT);
}


TEST_F(SynchronizerTestSuite, DryCaptureCanPrime)
{
  driver->inject(Dispatch<int, int>{10, 10});
  follower1->inject(Dispatch<int, double>{0, 2.0});
  follower1->inject(Dispatch<int, double>{9, 2.0});
  follower2->inject(Dispatch<int, std::string>{20, "ok"});

  const auto result = Synchronizer::dry_capture(std::forward_as_tuple(*driver, *follower1, *follower2), 0);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.state, State::PRIMED);
}


// Type we will use to as a data sequencing stamp
using sequencing_type = int;


// Sequencing stamp + data
template <typename T> using MyDispatch = Dispatch<sequencing_type, T>;


TEST(Synchronizer, UsageExampleExampleSingleThreaded)
{
  using namespace flow;

  // Type aliases for captors
  using NextType = driver::Next<MyDispatch<int>, NoLock>;
  using ClosestType = follower::ClosestBefore<MyDispatch<double>, NoLock>;
  using BeforeType = follower::Before<MyDispatch<std::string>, NoLock>;

  // Setup driving captor
  NextType next_driver;

  // Setup first following input captor
  ClosestType closest_follower{1 /*period*/, 0 /*delay offset*/};

  // Setup second following input captor
  BeforeType before_follower{1 /*delay offset*/};

  // Add some data
  for (int n = 0; n < 20; ++n)
  {
    next_driver.inject(n, n);
    closest_follower.inject(n - 2, static_cast<double>(n) + 0.1234);
    before_follower.inject(n - 4, "flow" + std::to_string(n));
  }

  static constexpr std::size_t EXPECTED_SYNC_COUNT = 17;
  std::size_t sync_count = 0;

  // Synchronize
  bool working = true;
  while (working)
  {
    // Any iterable container can be used to capture data (even raw buffers!). Capturing through
    // iterators allows the end user to specify their own memory handling
    std::vector<MyDispatch<int>> next_driver_data;
    std::list<MyDispatch<double>> closest_follower_data;
    std::deque<MyDispatch<std::string>> before_follower_data;

    // Run data capture
    const auto result = Synchronizer::capture(
      std::forward_as_tuple(next_driver, closest_follower, before_follower),
      std::forward_as_tuple(
        std::back_inserter(next_driver_data),
        std::back_inserter(closest_follower_data),
        std::back_inserter(before_follower_data)));

    switch (result.state)
    {
    case State::PRIMED:
      // do stuff with synchronized data
      ++sync_count;
      break;
    case State::ABORT:
      // could not synchronize with current data
      break;
    case State::RETRY:
      // need newer data to synchronize
      working = false;
      break;
    case State::TIMEOUT:  // multithreading only
      break;
    default:
      break;
    }
  }

  // Start a clean slate, if you need to
  Synchronizer::reset(std::forward_as_tuple(next_driver, closest_follower, before_follower));

  ASSERT_EQ(next_driver.size(), 0UL);
  ASSERT_EQ(closest_follower.size(), 0UL);
  ASSERT_EQ(before_follower.size(), 0UL);

  ASSERT_EQ(sync_count, EXPECTED_SYNC_COUNT);
}


// Flow + Multithreading
#include <condition_variable>
#include <mutex>
#include <thread>

TEST(Synchronizer, UsageExampleExampleMultiThreaded)
{
  using namespace flow;

  // Type aliases for captors
  using NextType = driver::Next<MyDispatch<int>, std::unique_lock<std::mutex>>;
  using ClosestType = follower::ClosestBefore<MyDispatch<double>, std::unique_lock<std::mutex>>;
  using BeforeType = follower::Before<MyDispatch<std::string>, std::unique_lock<std::mutex>>;

  // Setup driving captor
  NextType next_driver;

  // Setup first following input captor
  ClosestType closest_follower{1 /*period*/, 0 /*delay offset*/};

  // Setup second following input captor
  BeforeType before_follower{1 /*delay offset*/};

  std::mutex progress_mutex;
  std::condition_variable progress_cv;

  static constexpr std::size_t EXPECTED_SYNC_COUNT = 17;
  std::size_t sync_count = 0;

  // Synchronize
  std::thread synchronizer_thread{[&] {
    bool working = true;
    while (working)
    {
      // Any iterable container can be used to capture data (even raw buffers!). Capturing through
      // iterators allows the end user to specify their own memory handling
      std::vector<MyDispatch<int>> next_driver_data;
      std::list<MyDispatch<double>> closest_follower_data;
      std::deque<MyDispatch<std::string>> before_follower_data;

      // Run data capture
      const auto result = Synchronizer::capture(
        std::forward_as_tuple(next_driver, closest_follower, before_follower),
        std::forward_as_tuple(
          std::back_inserter(next_driver_data),
          std::back_inserter(closest_follower_data),
          std::back_inserter(before_follower_data)));

      switch (result.state)
      {
      case State::PRIMED:
        // do stuff with synchronized data
        ++sync_count;
        if (sync_count == EXPECTED_SYNC_COUNT)
        {
          progress_cv.notify_one();
          working = false;
        }
        break;
      case State::ABORT:
        // could not synchronize with current data
        // normally we could keep working, but we want the test to end
        break;
      case State::RETRY:  // polling-mode only
        break;
      case State::TIMEOUT:
        break;
      default:
        break;
      }
    }
    progress_cv.notify_all();
  }};

  // Add some data
  for (int n = 0; n < 20; ++n)
  {
    next_driver.inject(n, n);
    closest_follower.inject(n - 2, static_cast<double>(n) + 0.1234);
    before_follower.inject(n - 4, "flow" + std::to_string(n));
  }

  // Wait for sync pogress
  std::unique_lock<std::mutex> lock{progress_mutex};
  progress_cv.wait(lock);

  // Cancel all data-waits; start a clean slate, if you need to
  Synchronizer::reset(std::forward_as_tuple(next_driver, closest_follower, before_follower));

  synchronizer_thread.join();

  ASSERT_EQ(next_driver.size(), 0UL);
  ASSERT_EQ(closest_follower.size(), 0UL);
  ASSERT_EQ(before_follower.size(), 0UL);

  ASSERT_EQ(sync_count, EXPECTED_SYNC_COUNT);
}

#endif  // DOXYGEN_SKIP
