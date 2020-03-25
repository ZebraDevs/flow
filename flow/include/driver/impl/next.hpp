/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_DRIVER_IMPL_NEXT_HPP
#define FLOW_DRIVER_IMPL_NEXT_HPP

namespace flow
{
namespace driver
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Next<DispatchT, LockPolicyT, AllocatorT>::Next(const AllocatorT& alloc) noexcept(false) :
  PolicyType{alloc}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Next<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(OutputDispatchIteratorT output,
                                                                    CaptureRange<stamp_type>& range)
{
  // Abort if there are no queued dispatches
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // Set time range
  range.lower_stamp = PolicyType::queue_.oldest_stamp();
  range.upper_stamp = range.lower_stamp;

  // Get next element
  *(output++) = PolicyType::queue_.pop();
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Next<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_DRIVER_IMPL_NEXT_HPP
