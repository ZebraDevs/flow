/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 * 
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
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

template<typename CaptorT>
CaptorInterface<CaptorT>::CaptorInterface(const size_type capacity) :
  capacity_{capacity}
{}


template<typename CaptorT>
CaptorInterface<CaptorT>::CaptorInterface(const size_type capacity, const DispatchAllocatorType& alloc) :
  capacity_{capacity},
  queue_{alloc}
{}


template<typename CaptorT>
template<typename... InsertArgTs>
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
