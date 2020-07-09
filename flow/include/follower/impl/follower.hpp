/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_FOLLOWER_HPP
#define FLOW_FOLLOWER_IMPL_FOLLOWER_HPP

// C++ Standard Library
#include <utility>

// Flow
#include <flow/captor_state.h>

namespace flow
{

template<typename PolicyT>
Follower<PolicyT>::Follower() :
  CaptorType{},
  loopback_enabled_{false},
  loopback_primed_{true}
{}


template<typename PolicyT>
Follower<PolicyT>::Follower(const DispatchAllocatorType& alloc) :
  CaptorType{alloc},
  loopback_enabled_{false},
  loopback_primed_{true}
{}


template<typename PolicyT>
void Follower<PolicyT>::setLoopBackMode(bool enabled)
{
  loopback_enabled_ = enabled;
  loopback_primed_ = !enabled;
}


template<typename PolicyT>
template<typename OutputDispatchIteratorT>
State Follower<PolicyT>::capture_policy_impl(OutputDispatchIteratorT&& output,
                                             const CaptureRange<stamp_type>& range)
{
  // Get prime-attempt result
  const State st = derived()->capture_follower_impl(std::forward<OutputDispatchIteratorT>(output), range);

  // Update loop-back initialization flag
  loopback_primed_ = loopback_primed_ or st == State::PRIMED;

  // Return primed if initial loop-back priming has not occurred yet
  return loopback_primed_ ? st : State::PRIMED;
}


template<typename PolicyT>
State Follower<PolicyT>::dry_capture_policy_impl(const CaptureRange<stamp_type>& range)
{
  return derived()->dry_capture_follower_impl(range);
}


template<typename PolicyT>
void Follower<PolicyT>::abort_policy_impl(const stamp_type& t_abort)
{
  // Reset loop-back
  setLoopBackMode(loopback_enabled_);

  derived()->abort_follower_impl(t_abort);
}


template<typename PolicyT>
void Follower<PolicyT>::reset_policy_impl()
{
  // Reset loop-back
  setLoopBackMode(loopback_enabled_);

  derived()->reset_follower_impl();
}

}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_FOLLOWER_HPP
