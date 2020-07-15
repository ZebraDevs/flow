// Copyright (C) 2020, Fetch Robotics Inc.
//
// This file is part of Flow.
//
// Flow is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Flow is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Flow.  If not, see <https://www.gnu.org/licenses/>.

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

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Before<DispatchT, LockPolicyT, AllocatorT>::Before(const offset_type& delay) : delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Before<DispatchT, LockPolicyT, AllocatorT>::Before(const offset_type& delay, const AllocatorT& alloc) :
    PolicyType{alloc},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State Before<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(
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


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State Before<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range) const
{
  const stamp_type boundary = range.upper_stamp - delay_;
  return (PolicyType::queue_.empty() or PolicyType::queue_.newest_stamp() < boundary) ? State::RETRY : State::PRIMED;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Before<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_BEFORE_HPP
