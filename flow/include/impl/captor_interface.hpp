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


namespace detail
{

/**
 * @brief Returns stamp associated with a captor
 * @tparam captor  captor object type (has associated CaptorTraits)
 *
 * @return stamp
 */
template<typename CaptorT>
constexpr typename CaptorTraits<CaptorT>::stamp_type check_stamp_type()
{
  return StampTraits<typename CaptorTraits<CaptorT>::stamp_type>::min();
}


/**
 * @brief Returns stamp associated with a several captor
 * @tparam c1  captor object type (has associated CaptorTraits)
 * @tparam c2  captor object type (has associated CaptorTraits)
 * @tparam other  more captor object types
 *
 * @return stamp
 *
 * @warning All captors must have the same stamp type, or this will fail at compile time
 */
template<typename CaptorT1, typename CaptorT2, typename... CaptorTs>
constexpr typename CaptorTraits<CaptorT1>::stamp_type check_stamp_type()
{
  using stamp_type_1 = decltype(check_stamp_type<CaptorT1>());
  using stamp_type_2 = decltype(check_stamp_type<CaptorT2, CaptorTs...>());

  static_assert(std::is_same<stamp_type_1, stamp_type_2>(),
                "All captors must have the same associated dispatch 'stamp_type'");

  return check_stamp_type<CaptorT1>();
}

}  // namespace detail
}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_INTERFACE_HPP
