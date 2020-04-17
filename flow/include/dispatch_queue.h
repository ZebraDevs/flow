/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file dispatch_queue.h
 */
#ifndef FLOW_DISPATCH_QUEUE_H
#define FLOW_DISPATCH_QUEUE_H

// C++ Standard Library
#include <deque>
#include <utility>

// Flow
#include <flow/dispatch.h>

namespace flow
{

/**
 * @brief Dispatch queuing data structure
 *
 *        FILO-type queue which orders data by sequence stamp, from oldest to newest. Provides
 *        useful methods for extracting data within stamped/counted ranges
 *
 *        This container is based on an <code>std::deque</code> and is allocator-aware
 *
 * @tparam DispatchT  data dipatch type
 * @tparam AllocatorT <code>DispatchT</code> allocator type
 */
template<typename DispatchT, typename AllocatorT = std::allocator<DispatchT>>
class DispatchQueue
{
public:
  /// Dispatch stamp type
  using stamp_type = typename DispatchTraits<DispatchT>::stamp_type;

  /// Dispatch data value type
  using value_type = typename DispatchTraits<DispatchT>::value_type;

  /// Underlying container alias
  using BaseContainerType = std::deque<DispatchT>;

  /// Sizing type alias
  using size_type = typename BaseContainerType::size_type;

  /// Iterator type for container Dispatch elements
  using const_iterator = typename BaseContainerType::const_iterator;

  /// Iterator type for container Dispatch elements
  using const_reverse_iterator = typename BaseContainerType::const_reverse_iterator;

  /**
   * @brief Default construtor
   */
  DispatchQueue() = default;

  /**
   * @brief Allocator construtor
   */
  explicit DispatchQueue(const AllocatorT& alloc);

  /**
   * @brief Returns the number queued elements
   */
  inline size_type size() const;

  /**
   * @brief Returns the number queued elements
   */
  inline bool empty() const;

  /**
   * @brief Returns first iterator to underlying ordered data structure
   * @return <code>const_iterator</code> to first Dispatch resource
   */
  inline const_iterator begin() const
  {
    return queue_.cbegin();
  }

  /**
   * @brief Returns last iterator to underlying ordered data structure
   * @return <code>const_iterator</code> to one element past Dispatch resource
   */
  inline const_iterator end() const
  {
    return queue_.cend();
  }

  /**
   * @brief Returns first iterator to reversed underlying ordered data structure
   * @return <code>const_reverse_iterator</code> to first Dispatch resource
   */
  inline const_reverse_iterator rbegin() const
  {
    return queue_.crbegin();
  }

  /**
   * @brief Returns last iterator to reversed underlying ordered data structure
   * @return <code>const_reverse_iterator</code> to one element past Dispatch resource
   */
  inline const_reverse_iterator rend() const
  {
    return queue_.crend();
  }

  /**
   * @brief Sequencing stamp associated with the oldest data
   * @return sequencing stamp of first-queued Dispatch
   *
   * @warning Undefined behavior when <code>empty() == true</code>
   */
  inline const stamp_type& oldest_stamp() const
  {
    return queue_.front().stamp();
  }

  /**
   * @brief Sequencing stamp associated with the newest data
   * @return sequencing stamp of last-queued Dispatch
   *
   * @warning Undefined behavior when <code>empty() == true</code>
   */
  inline const stamp_type& newest_stamp() const
  {
    return queue_.back().stamp();
  }

  /**
   * @brief Removes the oldest element and returns associated Dispatch
   * @return oldest element
   */
  inline DispatchT pop();

  /**
   * @brief Removes all data from queue
   */
  inline void clear();

  /**
   * @brief Removes data with stamp older than reference sequence stamp
   * @param stamp  lower bound on container sequence stamp
   */
  inline void remove_before(const stamp_type& t);

  /**
   * @brief Removes data with stamp older than or equal to some reference sequence stamp
   * @param stamp  lower bound on container sequence stamp
   */
  inline void remove_at_before(const stamp_type& t);

  /**
   * @brief Removes oldest data until queue has less than or equal to N-elements
   * @param n  lower bound on total container size
   */
  inline void shrink_to_fit(size_type n);

  /**
   * @brief Inserts data in sequence stamp order as Dispatch (duplicates allowed)
   * @param dispatch_args  dispatch constructor args
   */
  template<typename... DispatchConstructorArgTs>
  inline void insert(DispatchConstructorArgTs&&... dispatch_args);

private:
  /// Queued data dispatches
  BaseContainerType queue_;
};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/dispatch_queue.hpp>

#endif  // FLOW_DISPATCH_QUEUE_H
