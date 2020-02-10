/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <chrono>
#include <iterator>
#include <memory>
#include <queue>
#include <string>
#include <thread>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/synchronizer.h>
#include <flow/followers.h>
#include <flow/drivers.h>

using namespace flow;

/// Tests the Synchronizer class
class SynchronizerTestSuite : public ::testing::Test
{
public:
  using Driver = driver::Next<Dispatch<int, int>, NoLock>;
  using Follower1 = follower::ClosestBefore<Dispatch<int, double>, NoLock>;
  using Follower2 = follower::Before<Dispatch<int, std::string>, NoLock>;

  using Block = Synchronizer<Driver,
                           Follower1,
                           Follower2>;

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


TEST_F(SynchronizerTestSuite, Reset)
{
  // Create synchronizer
  Synchronizer<Driver, Follower1, Follower2> synchronizer;

  synchronizer.reset(std::forward_as_tuple(*driver, *follower1, *follower2));
}


TEST_F(SynchronizerTestSuite, CannotPrimeRetry)
{
  // Create synchronizer
  Synchronizer<Driver, Follower1, Follower2> synchronizer;

  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = synchronizer.capture(std::forward_as_tuple(*driver, *follower1, *follower2),
                                           std::forward_as_tuple(std::back_inserter(driver_output_data),
                                                                 std::back_inserter(follower1_output_data),
                                                                 std::back_inserter(follower2_output_data)));

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::RETRY);
}


TEST_F(SynchronizerTestSuite, CannotPrimeAbort)
{
  // Create synchronizer
  Synchronizer<Driver, Follower1, Follower2> synchronizer;

  driver->inject(Dispatch<int, int>{1, 1});
  follower1->inject(Dispatch<int, double>{7, 2.0});
  follower2->inject(Dispatch<int, std::string>{0, "ok"});

  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = synchronizer.capture(std::forward_as_tuple(*driver, *follower1, *follower2),
                                           std::forward_as_tuple(std::back_inserter(driver_output_data),
                                                                 std::back_inserter(follower1_output_data),
                                                                 std::back_inserter(follower2_output_data)));

  ASSERT_FALSE(result);
  ASSERT_EQ(result.state, State::ABORT);
}


TEST_F(SynchronizerTestSuite, CanPrime)
{
  // Create synchronizer
  Synchronizer<Driver, Follower1, Follower2> synchronizer;

  driver->inject(Dispatch<int, int>{10, 10});
  follower1->inject(Dispatch<int, double>{0, 2.0});
  follower1->inject(Dispatch<int, double>{10, 2.0});
  follower2->inject(Dispatch<int, std::string>{20, "ok"});

  std::vector<Dispatch<int, int>> driver_output_data;
  std::vector<Dispatch<int, double>> follower1_output_data;
  std::vector<Dispatch<int, std::string>> follower2_output_data;

  const auto result = synchronizer.capture(std::forward_as_tuple(*driver, *follower1, *follower2),
                                           std::forward_as_tuple(std::back_inserter(driver_output_data),
                                                                 std::back_inserter(follower1_output_data),
                                                                 std::back_inserter(follower2_output_data)));

  ASSERT_TRUE(result);
  ASSERT_EQ(result.state, State::PRIMED);

  ASSERT_FALSE(driver_output_data.empty());
  ASSERT_FALSE(follower1_output_data.empty());
  ASSERT_TRUE(follower2_output_data.empty());
}

#endif  // DOXYGEN_SKIP
