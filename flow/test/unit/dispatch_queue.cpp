/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef DOXYGEN_SKIP

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/dispatch_queue.h>

using namespace flow;


TEST(DispatchQueue, DefaultIsEmpty)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  EXPECT_TRUE(queue.empty());
}


TEST(DispatchQueue, DefaultIteratorsEqual)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  ASSERT_TRUE(queue.empty());
  EXPECT_TRUE(queue.begin() == queue.end());
}


TEST(DispatchQueue, Insert)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;

  queue.insert(DispatchType{t0, 1});

  ASSERT_FALSE(queue.empty());
  EXPECT_EQ(queue.size(), 1u);
}


TEST(DispatchQueue, InsertDispatchType)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;

  queue.insert(DispatchType{t0, 1});

  ASSERT_FALSE(queue.empty());
  EXPECT_EQ(queue.size(), 1u);
}


TEST(DispatchQueue, OldestStamp)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t_oldest{0};

  queue.insert(DispatchType{t_oldest, 1});
  queue.insert(DispatchType{t_oldest + 1, 2});
  queue.insert(DispatchType{t_oldest + 2, 3});

  ASSERT_FALSE(queue.empty());
  EXPECT_EQ(queue.oldest_stamp(), t_oldest);
}


TEST(DispatchQueue, NewestStamp)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t_oldest{0};

  queue.insert(DispatchType{t_oldest, 1});
  queue.insert(DispatchType{t_oldest + 1, 2});
  queue.insert(DispatchType{t_oldest + 2, 3});

  ASSERT_FALSE(queue.empty());
  EXPECT_EQ(queue.newest_stamp(), t_oldest + 2);
}


TEST(DispatchQueue, ShrinkToFitSmallerThanOriginal)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t1, 1});
  queue.insert(DispatchType{t2, 1});

  queue.shrink_to_fit(1);

  EXPECT_EQ(queue.size(), 1u);
}


TEST(DispatchQueue, ShrinkToFitLargerThanOriginal)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t1, 1});
  queue.insert(DispatchType{t2, 1});

  queue.shrink_to_fit(4);

  EXPECT_EQ(queue.size(), 3u);
}


TEST(DispatchQueue, RemoveNone)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t1, 1});
  queue.insert(DispatchType{t2, 1});

  queue.remove_before(t0);

  EXPECT_EQ(queue.size(), 3u);
}


TEST(DispatchQueue, RemoveAll)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t1, 1});
  queue.insert(DispatchType{t2, 1});

  queue.remove_before(t2 + 1);

  EXPECT_TRUE(queue.empty());
}


TEST(DispatchQueue, InsertDuplicateTimeBehavior)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t0, 2});
  queue.insert(DispatchType{t0, 3});

  ASSERT_EQ(queue.size(), 1u);

  const auto p0 = queue.pop();

  EXPECT_EQ(p0.stamp(), t0);
  EXPECT_EQ(p0.data(), 1);
}


TEST(DispatchQueue, InsertOrdered)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t1, 1});
  queue.insert(DispatchType{t2, 1});

  EXPECT_EQ(queue.size(), 3u);

  const auto p0 = queue.pop();
  const auto p1 = queue.pop();
  const auto p2 = queue.pop();

  EXPECT_LT(p0.stamp(), p1.stamp());
  EXPECT_LT(p1.stamp(), p2.stamp());
}


TEST(DispatchQueue, InsertUnordered)
{
  using DispatchType = Dispatch<int, int>;

  DispatchQueue<DispatchType> queue;

  const int t0 = 0;
  const int t1 = t0 + 1;
  const int t2 = t1 + 1;

  queue.insert(DispatchType{t0, 1});
  queue.insert(DispatchType{t2, 1});
  queue.insert(DispatchType{t1, 1});

  EXPECT_EQ(queue.size(), 3u);

  const auto p0 = queue.pop();
  const auto p1 = queue.pop();
  const auto p2 = queue.pop();

  EXPECT_LT(p0.stamp(), p1.stamp());
  EXPECT_LT(p1.stamp(), p2.stamp());
}


#endif  // DOXYGEN_SKIP
