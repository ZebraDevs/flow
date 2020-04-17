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

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Ranged<DispatchT, LockPolicyT, AllocatorT>::Ranged(const offset_type& period, const offset_type& delay) :
  PolicyType{},
  period_{period},
  delay_{delay}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Ranged<DispatchT, LockPolicyT, AllocatorT>::Ranged(const offset_type& period,
                                                   const offset_type& delay,
                                                   const AllocatorT& alloc) :
  PolicyType{alloc},
  period_{period},
  delay_{delay}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Ranged<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT&& output,
                                                                       const CaptureRange<stamp_type>& range)
{
  // Abort early on empty queue
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // First first element after/at lower offset
  const auto past_first = std::find_if(
    PolicyType::queue_.begin(),
    PolicyType::queue_.end(),
    [offset_lower_stamp=(range.lower_stamp - delay_)](const auto& dispatch) {return dispatch.stamp() >= offset_lower_stamp;});

  // If we are at the start of the available range, then all elements after this one will be after the valid range
  if (past_first == PolicyType::queue_.begin())
  {
    return State::ABORT;
  }

  // Find initial end position iterator
  const auto before_last = std::find_if_not(
    past_first == PolicyType::queue_.end() ? PolicyType::queue_.begin() : past_first,
    PolicyType::queue_.end(),
    [offset_upper_stamp=(range.upper_stamp - delay_)](const auto& dispatch) {return dispatch.stamp() <= offset_upper_stamp;});

  // If the end of our range
  if (before_last == PolicyType::queue_.end())
  {
    return State::RETRY;
  }

  // Get capture iterator range
  const auto first = std::prev(past_first);
  const auto last = std::next(before_last);

  // Copy captured data over range
  std::copy(first, last, std::forward<OutputDispatchIteratorT>(output));

  // Remove data before first captured element
  PolicyType::queue_.remove_before(first->stamp());

  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Ranged<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_ - period_ - period_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_RANGED_HPP
