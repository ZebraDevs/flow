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

#ifndef FLOW_CAPTURE_IMPL_CAPTOR_INTERFACE_HPP
#define FLOW_CAPTURE_IMPL_CAPTOR_INTERFACE_HPP

// C++ Standard Library
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

namespace flow
{

template <typename CaptorT> CaptorInterface<CaptorT>::CaptorInterface(const size_type capacity) : capacity_{capacity} {}


template <typename CaptorT>
CaptorInterface<CaptorT>::CaptorInterface(const size_type capacity, const DispatchAllocatorType& alloc) :
    capacity_{capacity},
    queue_{alloc}
{}


template <typename CaptorT>
template <typename... InsertArgTs>
void CaptorInterface<CaptorT>::insert_and_limit(InsertArgTs&&... args)
{
  queue_.insert(std::forward<InsertArgTs>(args)...);
  if (capacity_)
  {
    queue_.shrink_to_fit(capacity_);
  }
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_INTERFACE_HPP
