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

template <typename PolicyT> Follower<PolicyT>::Follower() : CaptorType{} {}


template <typename PolicyT> Follower<PolicyT>::Follower(const DispatchAllocatorType& alloc) : CaptorType{alloc} {}


template <typename PolicyT>
template <typename OutputDispatchIteratorT>
State Follower<PolicyT>::capture_policy_impl(OutputDispatchIteratorT&& output, const CaptureRange<stamp_type>& range)
{
  return derived()->capture_follower_impl(std::forward<OutputDispatchIteratorT>(output), range);
}


template <typename PolicyT> State Follower<PolicyT>::dry_capture_policy_impl(const CaptureRange<stamp_type>& range)
{
  return derived()->dry_capture_follower_impl(range);
}


template <typename PolicyT> void Follower<PolicyT>::abort_policy_impl(const stamp_type& t_abort)
{
  derived()->abort_follower_impl(t_abort);
}


template <typename PolicyT> void Follower<PolicyT>::reset_policy_impl() { derived()->reset_follower_impl(); }

}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_FOLLOWER_HPP
