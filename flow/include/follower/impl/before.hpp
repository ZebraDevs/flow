/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_BEFORE_HPP
#define FLOW_FOLLOWER_IMPL_BEFORE_HPP

// C++ Standard Library
#include <cstdint>
#include <mutex>
#include <stdexcept>


namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename ContainerT>
Before<DispatchT, LockPolicyT, ContainerT>::Before(const offset_type& delay) : delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename ContainerT>
Before<DispatchT, LockPolicyT, ContainerT>::Before(const offset_type& delay, const ContainerT& container) :
    PolicyType{container},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename ContainerT>
template <typename OutputDispatchIteratorT>
State Before<DispatchT, LockPolicyT, ContainerT>::capture_follower_impl(
  OutputDispatchIteratorT output,
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
  if (PolicyType::queue_.newest_stamp() < boundary)
  {
    return State::RETRY;
  }

  // Collect all the messages that are earlier than the first driving message's
  // timestamp minus the delay
  while (!PolicyType::queue_.empty() and PolicyType::queue_.oldest_stamp() < boundary)
  {
    *(output++) = PolicyType::queue_.pop();
  }

  // Remove all elements before delayed boundary
  PolicyType::queue_.remove_before(boundary);
  return State::PRIMED;
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT>
State Before<DispatchT, LockPolicyT, ContainerT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range) const
{
  const stamp_type boundary = range.upper_stamp - delay_;
  return (PolicyType::queue_.empty() or PolicyType::queue_.newest_stamp() < boundary) ? State::RETRY : State::PRIMED;
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT>
void Before<DispatchT, LockPolicyT, ContainerT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_BEFORE_HPP
