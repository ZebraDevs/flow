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

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Throttled<DispatchT, LockPolicyT, AllocatorT>::Throttled(const offset_type throttle_period) noexcept(false) :
  PolicyType{},
  throttle_period_{throttle_period}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Throttled<DispatchT, LockPolicyT, AllocatorT>::Throttled(const offset_type throttle_period, const AllocatorT& alloc) noexcept(false) :
  PolicyType{alloc},
  throttle_period_{throttle_period}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Throttled<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(OutputDispatchIteratorT output,
                                                                         CaptureRange<stamp_type>& range)
{
  // Abort if there are no queued dispatches
  if (PolicyType::queue_.size() < 2)
  {
    return State::RETRY;
  }

  // Get oldest time stamp
  const auto oldest_stamp = PolicyType::queue_.oldest_stamp();

  // Capture message which falls within throttling period
  auto itr = std::next(PolicyType::queue_.begin());
  while (itr != PolicyType::queue_.end())
  {
    if (itr->stamp() - oldest_stamp >= throttle_period_)
    {
      range.lower_stamp = itr->stamp();
      range.upper_stamp = range.lower_stamp;

      *(output++) = *(itr);

      PolicyType::queue_.remove_before(itr->stamp());

      return State::PRIMED;
    }
    else
    {
      ++itr;
    }
  }
  return State::RETRY;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Throttled<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_DRIVER_IMPL_TRHOTTLED_HPP
