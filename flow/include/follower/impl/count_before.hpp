/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_COUNT_BEFORE_HPP
#define FLOW_FOLLOWER_IMPL_COUNT_BEFORE_HPP

// C++ Standard Library
#include <cstdint>
#include <stdexcept>


namespace flow
{
namespace follower
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
CountBefore<DispatchT, LockPolicyT, AllocatorT>::CountBefore(const size_type count, const offset_type& delay) :
  count_{count},
  delay_{delay}
{
  if (count_ == 0)
  {
    throw std::invalid_argument{"'count' cannot be 0"};
  }
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
CountBefore<DispatchT, LockPolicyT, AllocatorT>::CountBefore(const size_type count, const offset_type& delay, const AllocatorT& alloc) :
  PolicyType{alloc},
  count_{count},
  delay_{delay}
{
  if (count_ == 0)
  {
    throw std::invalid_argument{"'count' cannot be 0"};
  }
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State CountBefore<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT output,
                                                                             const CaptureRange<stamp_type>& range)
{
  // Retry if queue has no data
  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }

  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.upper_stamp - delay_;

  // Scroll to element boundary
  size_type before_boundary_count = 0;
  auto itr = PolicyType::queue_.begin();
  while (itr != PolicyType::queue_.end() and boundary > itr->stamp())
  {
    ++before_boundary_count;
    ++itr;
  }

  // Count elements after boundary
  const bool at_or_after_boundary_count = itr != PolicyType::queue_.end();

  if (before_boundary_count >= count_)
  {
    auto first_itr = std::prev(itr, count_);
    std::copy(first_itr, itr, output);
    PolicyType::queue_.remove_before(first_itr->stamp());
    return State::PRIMED;
  }
  else if (at_or_after_boundary_count)
  {
    return State::ABORT;
  }
  return State::RETRY;
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_COUNT_BEFORE_HPP
