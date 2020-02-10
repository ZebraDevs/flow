/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
#define FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP

// C++ Standard Library
#include <chrono>

namespace flow
{
namespace follower
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::ClosestBefore(const offset_type& period,
                                                                 const offset_type& delay) :
  PolicyType{},
  period_{period},
  delay_{delay}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::ClosestBefore(const offset_type& period,
                                                                 const offset_type& delay,
                                                                 const AllocatorT& alloc) :
  PolicyType{alloc},
  period_{period},
  delay_{delay}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT output,
                                                                               const CaptureRange<stamp_type>& range)
{
  // Retry if queue has no data
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.upper_stamp - delay_;

  // Retry if priming is not possible
  if (PolicyType::queue_.oldest_stamp() > boundary + period_)
  {
    return State::ABORT;
  }

  // Return messages on:
  // - next message stamp > boundary
  // - stamp + period > boundary
  auto curr_qitr = PolicyType::queue_.rbegin();
  while (curr_qitr != PolicyType::queue_.rend())
  {
    const auto prev_qitr = std::next(curr_qitr);
    if (((prev_qitr != PolicyType::queue_.rend()) and (prev_qitr->stamp() < boundary) and (curr_qitr->stamp() >= boundary)) or
        ((curr_qitr->stamp() < boundary) and (curr_qitr->stamp() + period_ >= boundary)))
    {
      *(output++) = *curr_qitr;
      break;
    }
    else
    {
      ++curr_qitr;
    }
  }

  // Remove all elements before delayed boundary
  PolicyType::queue_.remove_before(boundary);
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  // don't remove anything abort
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
