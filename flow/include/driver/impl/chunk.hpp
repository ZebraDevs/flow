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

#ifndef FLOW_DRIVER_IMPL_CHUNK_HPP
#define FLOW_DRIVER_IMPL_CHUNK_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace flow
{
namespace driver
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Chunk<DispatchT, LockPolicyT, AllocatorT>::Chunk(size_type size) noexcept(false) : PolicyType{}, chunk_size_{size}
{
  validate();
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Chunk<DispatchT, LockPolicyT, AllocatorT>::Chunk(size_type size, const AllocatorT& alloc) noexcept(false) :
    PolicyType{alloc},
    chunk_size_{size}
{
  validate();
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State Chunk<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(
  OutputDispatchIteratorT output,
  CaptureRange<stamp_type>& range)
{
  const State state = this->dry_capture_driver_impl(range);

  if (state == State::PRIMED)
  {
    // Collect dispatches
    std::copy_n(std::make_move_iterator(PolicyType::queue_.begin()), chunk_size_, output);

    // Remove captured elements
    for (size_type i = 0; i < chunk_size_; ++i)
    {
      PolicyType::queue_.pop();
    }
  }

  return state;
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State Chunk<DispatchT, LockPolicyT, AllocatorT>::dry_capture_driver_impl(CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.size() >= chunk_size_)
  {
    auto oldest_itr = PolicyType::queue_.begin();

    range.lower_stamp = get_stamp(*oldest_itr);
    range.upper_stamp = get_stamp(*std::next(oldest_itr, chunk_size_ - 1));

    return State::PRIMED;
  }
  else
  {
    return State::RETRY;
  }
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Chunk<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Chunk<DispatchT, LockPolicyT, AllocatorT>::validate() const noexcept(false)
{
  if (chunk_size_ == 0)
  {
    throw std::invalid_argument{"'chunk_size_' should be greater than zero"};
  }
}

}  // namespace driver
}  // namespace flow


#endif  // FLOW_DRIVER_IMPL_CHUNK_HPP
