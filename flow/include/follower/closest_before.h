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

#ifndef FLOW_FOLLOWER_CLOSEST_BEFORE_H
#define FLOW_FOLLOWER_CLOSEST_BEFORE_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures one element before the capture range lower bound, minus a delay period, within an expected period.
 *
 *        All older elements are removed.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 *
 * @warn ClosestBefore will behave non-deterministically if actual input period (difference between successive
 *       dispatch stamps) does not match the <code>period</code> argument specified on construction. For example,
 *       if <code>period</code> is too large, than multiple inputs could appear before the driving range, causing
 *       for different data on two or more iterations where the "latest" data was assumed to have been the same
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename AllocatorT = std::allocator<DispatchT>>
class ClosestBefore : public Follower<ClosestBefore<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<ClosestBefore>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<ClosestBefore>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param period  expected period between successive data element
   * @param delay  the delay with which to capture
   */
  ClosestBefore(const offset_type& period, const offset_type& delay);

  /**
   * @brief Setup constructor
   *
   * @param period  expected half-period between successive data elements
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  ClosestBefore(const offset_type& period, const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<ClosestBefore<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval ABORT   If next closest element has a sequencing stamp greater than <code>range.upper_stamp</code>
   * @retval PRIMED  If next closest element to is available
   * @retval RETRY   Element with sequence stamp greater than <code>range.upper_stamp</code> exists, and
   *                 there is a data element within the expected duration window before
   *                 <code>range.upper_stamp</code>
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT output, const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::dry_capture_policy_impl
   */
  inline State dry_capture_follower_impl(const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::abort_policy_impl
   */
  inline void abort_follower_impl(const stamp_type& t_abort);

  /**
   * @copydoc Follower::reset_policy_impl
   */
  inline void reset_follower_impl() noexcept(true) {}

  /// Expected update period
  offset_type period_;

  /// Capture delay
  offset_type delay_;
};

}  // namespace follower


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
struct CaptorTraits<follower::ClosestBefore<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
  ;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/closest_before.hpp>

#endif  // FLOW_FOLLOWER_CLOSEST_BEFORE_H
