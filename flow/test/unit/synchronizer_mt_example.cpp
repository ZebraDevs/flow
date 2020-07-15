// Copyright (C) 2020, Fetch Robotics Inc.
//
// This file is part of Flow.
//
// Flow is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Flow is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Flow.  If not, see <https://www.gnu.org/licenses/>.

#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <condition_variable>
#include <chrono>
#include <deque>
#include <iterator>
#include <list>
#include <mutex>
#include <thread>
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


// Type we will use to as a data sequencing stamp
using sequencing_type = int;


// Sequencing stamp + data
template <typename T> using MyDispatch = Dispatch<sequencing_type, T>;


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


TEST(Synchronizer, UsageExampleExampleMultiThreadedPolling)
{
  using namespace flow;

  using LockType = PollingLock<std::lock_guard<std::mutex>>;

  // Type aliases for captors
  using NextType = driver::Next<MyDispatch<int>, LockType>;
  using ClosestType = follower::ClosestBefore<MyDispatch<double>, LockType>;
  using BeforeType = follower::Before<MyDispatch<std::string>, LockType>;

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

      // Do a little sleep
      std::this_thread::sleep_for(std::chrono::milliseconds{1});

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
