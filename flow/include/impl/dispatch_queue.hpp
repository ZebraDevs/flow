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

#ifndef FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
#define FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP

// C++ Standard Library
#include <iterator>

// Flow
#include <flow/dispatch.h>

namespace flow
{

template <typename DispatchT, typename AllocatorT>
DispatchQueue<DispatchT, AllocatorT>::DispatchQueue(const AllocatorT& alloc) : queue_{alloc}
{}

template <typename DispatchT, typename AllocatorT>
typename DispatchQueue<DispatchT, AllocatorT>::size_type DispatchQueue<DispatchT, AllocatorT>::size() const
{
  return queue_.size();
}


template <typename DispatchT, typename AllocatorT> bool DispatchQueue<DispatchT, AllocatorT>::empty() const
{
  return !size();
}


template <typename DispatchT, typename AllocatorT>
template <typename... DispatchConstructorArgTs>
void DispatchQueue<DispatchT, AllocatorT>::insert(DispatchConstructorArgTs&&... dispatch_args)
{
  DispatchT dispatch{std::forward<DispatchConstructorArgTs>(dispatch_args)...};

  // If data to add is ordered with respect to current queue,
  // add to back (as newest element)
  if (queue_.empty() or (get_stamp(queue_.back()) < get_stamp(dispatch)))
  {
    queue_.emplace_back(std::move(dispatch));
    return;
  }

  // Find next best placement
  auto qitr = queue_.end();
  while (get_stamp(*(--qitr)) > get_stamp(dispatch))
  {
    if (qitr == queue_.begin())
    {
      queue_.emplace_front(std::move(dispatch));
      return;
    }
  }

  // Insert only if this element does not duplicate an existing element
  if (get_stamp(*qitr) != get_stamp(dispatch))
  {
    queue_.emplace(std::next(qitr), std::move(dispatch));
  }
}


template <typename DispatchT, typename AllocatorT> DispatchT DispatchQueue<DispatchT, AllocatorT>::pop()
{
  const auto retval = std::move(queue_.front());
  queue_.pop_front();
  return retval;
}


template <typename DispatchT, typename AllocatorT> void DispatchQueue<DispatchT, AllocatorT>::clear()
{
  queue_.clear();
}


template <typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::remove_before(const stamp_type& t)
{
  while (!queue_.empty() and get_stamp(queue_.front()) < t)
  {
    queue_.pop_front();
  }
}


template <typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::remove_at_before(const stamp_type& t)
{
  while (!queue_.empty() and get_stamp(queue_.front()) <= t)
  {
    queue_.pop_front();
  }
}


template <typename DispatchT, typename AllocatorT> void DispatchQueue<DispatchT, AllocatorT>::shrink_to_fit(size_type n)
{
  while (queue_.size() > n)
  {
    queue_.pop_front();
  }
}


template <typename DispatchT, typename AllocatorT>
AllocatorT DispatchQueue<DispatchT, AllocatorT>::get_allocator() const noexcept
{
  return queue_.get_allocator();
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
