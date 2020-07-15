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

#ifndef FLOW_FOLLOWER_IMPL_FOLLOWER_HPP
#define FLOW_FOLLOWER_IMPL_FOLLOWER_HPP

// C++ Standard Library
#include <utility>

// Flow
#include <flow/captor_state.h>

namespace flow
{

template <typename PolicyT>
Follower<PolicyT>::Follower() : CaptorType{}
{}


template <typename PolicyT>
Follower<PolicyT>::Follower(const DispatchAllocatorType& alloc) :
    CaptorType{alloc}
{}


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


template <typename PolicyT> void Follower<PolicyT>::reset_policy_impl()
{
  derived()->reset_follower_impl();
}

}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_FOLLOWER_HPP
