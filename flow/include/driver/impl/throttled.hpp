/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_DRIVER_IMPL_TRHOTTLED_HPP
#define FLOW_DRIVER_IMPL_TRHOTTLED_HPP

// C++ Standard Library
#include <iterator>

namespace flow
{
namespace driver
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Throttled<DispatchT, LockPolicyT, AllocatorT>::Throttled(const offset_type throttle_period) noexcept(false) :
    PolicyType{},
    throttle_period_{throttle_period},
    previous_stamp_{StampTraits<stamp_type>::min()}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Throttled<DispatchT, LockPolicyT, AllocatorT>::Throttled(
  const offset_type throttle_period,
  const AllocatorT& alloc) noexcept(false) :
    PolicyType{alloc},
    throttle_period_{throttle_period},
    previous_stamp_{StampTraits<stamp_type>::min()}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State Throttled<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(
  OutputDispatchIteratorT output,
  CaptureRange<stamp_type>& range)
{
  const State state = this->dry_capture_driver_impl(range);

  if (state == State::PRIMED)
  {
    // Remove all elements before lower stamp, first
    PolicyType::queue_.remove_before(range.lower_stamp);

    // Cache previous stamp
    previous_stamp_ = range.lower_stamp;

    // Get next element and remove
    *(output++) = PolicyType::queue_.pop();
  }

  return state;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State Throttled<DispatchT, LockPolicyT, AllocatorT>::dry_capture_driver_impl(CaptureRange<stamp_type>& range) const
{
  for (const auto& dispatch : PolicyType::queue_)
  {
    if (
      previous_stamp_ == StampTraits<stamp_type>::min() or (get_stamp(dispatch) - previous_stamp_) >= throttle_period_)
    {
      range.lower_stamp = get_stamp(dispatch);
      range.upper_stamp = range.lower_stamp;

      return State::PRIMED;
    }
  }
  return State::RETRY;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Throttled<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Throttled<DispatchT, LockPolicyT, AllocatorT>::reset_driver_impl()
{
  previous_stamp_ = StampTraits<stamp_type>::min();
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_DRIVER_IMPL_TRHOTTLED_HPP
