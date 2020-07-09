/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
#define FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP

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
  const State state = this->dry_capture_follower_impl(range);

  if (state == State::PRIMED)
  {
    // When primed, next element should be captured without removal
    *(output++) = *PolicyType::queue_.begin();
  }

  return state;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
State ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range)
{
  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - delay_;

  // Starting from the oldest data, return on when data found within periodic window
  auto capture_itr = PolicyType::queue_.end();
  for (auto itr = PolicyType::queue_.begin(); itr != PolicyType::queue_.end(); ++itr)
  {
    if (itr->stamp() >= boundary)
    {
      // If oldest element is far ahead of boundary, abort
      return State::ABORT;
    }
    else if (itr->stamp() + period_ >= boundary)
    {
      // If oldest element is first within period, collect
      capture_itr = itr;
      break;
    }
  }

  // Check if inputs are capturable
  // NOTE: If no capture input was set, then all data is at or after boundary
  if (capture_itr != PolicyType::queue_.end())
  {
    PolicyType::queue_.remove_before(capture_itr->stamp());
    return State::PRIMED;
  }
  else
  {
    return State::RETRY;
  }
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_ - period_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
