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

#ifndef FLOW_FOLLOWER_RANGED_H
#define FLOW_FOLLOWER_RANGED_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures one one element before the capture range lower bound; one element after the capture range upper
 * bound.
 *
 *        All elements in between are also captured. All older elements are removed.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename AllocatorT = std::allocator<DispatchT>>
class Ranged : public Follower<Ranged<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Ranged>::stamp_type;

  /// Integer size type
  using size_type = typename CaptorTraits<Ranged>::size_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<Ranged>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param delay  the delay with which to capture
   */
  Ranged(const offset_type& delay);

  /**
   * @brief Setup constructor
   *
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  Ranged(const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<Ranged<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval ABORT   If N-elements do not exist before <code>range.lower_stamp</code>
   * @retval PRIMED  If N-elements exist before <code>range.lower_stamp</code> and
   *                 M-elements exist after <code>range.upper_stamp</code>
   * @retval RETRY   If N-elements exist before <code>range.lower_stamp</code> but
   *                 M-elements do not exist after <code>range.upper_stamp</code>
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT&& output, const CaptureRange<stamp_type>& range);

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

  /**
   * @brief Finds iterator after first in capture sequence
   *
   * @param range  data capture/sequencing range
   */
  inline auto find_after_first(const CaptureRange<stamp_type>& range) const;

  /**
   * @brief Finds iterator before last in capture sequence
   *
   * @param range  data capture/sequencing range
   */
  template <typename QueueIteratorT>
  inline auto find_before_last(const CaptureRange<stamp_type>& range, const QueueIteratorT after_first) const;

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
struct CaptorTraits<follower::Ranged<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
  ;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/ranged.hpp>

#endif  // FLOW_FOLLOWER_RANGED_H
