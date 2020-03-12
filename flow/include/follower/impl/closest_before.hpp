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
  const stamp_type boundary = range.lower_stamp - delay_;

  // Starting from the oldest data, return on when data found within periodic windowp
  auto capture_itr = PolicyType::queue_.end();
  for (auto itr = PolicyType::queue_.begin(); itr != PolicyType::queue_.end(); ++itr)
  {
    if (itr->stamp() >= boundary)
    {
      break;
    }
    else if (itr->stamp() + period_ >= boundary)
    {
      capture_itr = itr;
      break;
    }
  }

  // Check if inputs were captured
  // NOTE: If no capture iput was set, then all data is at or after boundary
  if (capture_itr == PolicyType::queue_.end())
  {
    return State::ABORT;
  }

  // Set captured data
  *output = *capture_itr;

  // Remove old data
  PolicyType::queue_.remove_before(capture_itr->stamp());

  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_ - period_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
