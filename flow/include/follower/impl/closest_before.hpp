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

#ifndef FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
#define FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::ClosestBefore(const offset_type& period, const offset_type& delay) :
    PolicyType{},
    period_{period},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::ClosestBefore(
  const offset_type& period,
  const offset_type& delay,
  const AllocatorT& alloc) :
    PolicyType{alloc},
    period_{period},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(
  OutputDispatchIteratorT output,
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


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(
  const CaptureRange<stamp_type>& range)
{
  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - delay_;

  // Starting from the oldest data, return on when data found within periodic window
  auto capture_itr = PolicyType::queue_.end();
  for (auto itr = PolicyType::queue_.begin(); itr != PolicyType::queue_.end(); ++itr)
  {
    if (get_stamp(*itr) >= boundary)
    {
      // If oldest element is far ahead of boundary, abort
      return State::ABORT;
    }
    else if (get_stamp(*itr) + period_ >= boundary)
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
    PolicyType::queue_.remove_before(get_stamp(*capture_itr));
    return State::PRIMED;
  }
  else
  {
    return State::RETRY;
  }
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void ClosestBefore<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_ - period_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_CLOSEST_BEFORE_HPP
