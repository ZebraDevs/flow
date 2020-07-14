/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_RANGED_HPP
#define FLOW_FOLLOWER_IMPL_RANGED_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Ranged<DispatchT, LockPolicyT, AllocatorT>::Ranged(const offset_type& delay) : PolicyType{}, delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Ranged<DispatchT, LockPolicyT, AllocatorT>::Ranged(const offset_type& delay, const AllocatorT& alloc) :
    PolicyType{alloc},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State Ranged<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(
  OutputDispatchIteratorT&& output,
  const CaptureRange<stamp_type>& range)
{
  // Abort early on empty queue
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // Get iterator to element after first in capture sequence
  const auto after_first_itr = this->find_after_first(range);

  // If we are at the start of the available range, then all elements after this one will be after the valid range
  if (after_first_itr == PolicyType::queue_.begin())
  {
    return State::ABORT;
  }

  // Find initial end position iterator
  const auto before_last_itr = this->find_before_last(range, after_first_itr);

  // If the end of our range
  if (before_last_itr == PolicyType::queue_.end())
  {
    return State::RETRY;
  }

  // Get capture iterator range
  const auto first_itr = std::prev(after_first_itr);
  const auto last_itr = std::next(before_last_itr);

  // Copy captured data over range
  std::copy(first_itr, last_itr, std::forward<OutputDispatchIteratorT>(output));

  // Remove data before first captured element
  PolicyType::queue_.remove_before(first_itr->stamp());

  return State::PRIMED;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State Ranged<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range)
{
  // Abort early on empty queue
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // Get iterator to element after first in capture sequence
  const auto after_first_itr = this->find_after_first(range);

  // If we are at the start of the available range, then all elements after this one will be after the valid range
  if (after_first_itr == PolicyType::queue_.begin())
  {
    return State::ABORT;
  }
  // If the end of our range
  else if (this->find_before_last(range, after_first_itr) == PolicyType::queue_.end())
  {
    return State::RETRY;
  }
  else
  {
    // Remove data before first captured element
    PolicyType::queue_.remove_before(std::prev(after_first_itr)->stamp());
    return State::PRIMED;
  }
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
auto Ranged<DispatchT, LockPolicyT, AllocatorT>::find_after_first(const CaptureRange<stamp_type>& range) const
{
  return std::find_if(
    PolicyType::queue_.begin(),
    PolicyType::queue_.end(),
    [offset_lower_stamp = (range.lower_stamp - delay_)](const DispatchT& dispatch) {
      return dispatch.stamp() >= offset_lower_stamp;
    });
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename QueueIteratorT>
auto Ranged<DispatchT, LockPolicyT, AllocatorT>::find_before_last(
  const CaptureRange<stamp_type>& range,
  const QueueIteratorT after_first) const
{
  return std::find_if_not(
    after_first == PolicyType::queue_.end() ? PolicyType::queue_.begin() : after_first,
    PolicyType::queue_.end(),
    [offset_upper_stamp = (range.upper_stamp - delay_)](const DispatchT& dispatch) {
      return dispatch.stamp() <= offset_upper_stamp;
    });
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Ranged<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_RANGED_HPP
