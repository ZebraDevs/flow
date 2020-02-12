/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_EXACT_HPP
#define FLOW_FOLLOWER_IMPL_EXACT_HPP

// C++ Standard Library
#include <iterator>


namespace flow
{
namespace follower
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Exact<DispatchT, LockPolicyT, AllocatorT>::Exact(const AllocatorT& alloc) :
  PolicyType{alloc}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Exact<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT output,
                                                                       const CaptureRange<stamp_type>& range)
{
  // Remove all elements before leading time
  PolicyType::queue_.remove_before(range.lower_stamp);

  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }
  else if (PolicyType::queue_.oldest_stamp() > range.upper_stamp)
  {
    return State::ABORT;
  }

  // Assign matching element
  *(output++) = PolicyType::queue_.pop();
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Exact<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_EXACT_HPP
