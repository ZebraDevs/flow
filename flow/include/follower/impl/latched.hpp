/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_LATCHED_HPP
#define FLOW_FOLLOWER_IMPL_LATCHED_HPP

// C++ Standard Library
#include <chrono>

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Latched<DispatchT, LockPolicyT, AllocatorT>::Latched(const offset_type min_period) :
    PolicyType{},
    min_period_{min_period}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
Latched<DispatchT, LockPolicyT, AllocatorT>::Latched(const offset_type min_period, const AllocatorT& alloc) :
    PolicyType{alloc},
    min_period_{min_period}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State Latched<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(
  OutputDispatchIteratorT output,
  const CaptureRange<stamp_type>& range)
{
  const State state = this->dry_capture_follower_impl(range);

  if (state == State::PRIMED)
  {
    // When primed, latched element should be captured
    *(output++) = *latched_;
  }

  return state;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
State Latched<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range)
{
  if (PolicyType::queue_.empty())
  {
    if (latched_)
    {
      // If we have latched data, return that and report ready state
      return State::PRIMED;
    }
    else
    {
      // Retry if queue has no data and there is no latched data
      return State::RETRY;
    }
  }

  // The boundary before which data is are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - min_period_;

  // Retry if priming is not possible
  if (PolicyType::queue_.oldest_stamp() > boundary)
  {
    if (latched_)
    {
      // If we have latched data, return that and report ready state
      return State::PRIMED;
    }
    else
    {
      // Abort if queue has no data and there is no latched data
      return State::ABORT;
    }
  }

  // Find data closest to boundary, starting from oldest
  // curr_qitr will never start at queue_.end() due to previous empty check
  auto curr_qitr = PolicyType::queue_.begin();
  auto prev_qitr = curr_qitr;
  while (curr_qitr != PolicyType::queue_.end() and curr_qitr->stamp() <= boundary)
  {
    prev_qitr = curr_qitr;
    ++curr_qitr;
  }

  // Set latched element
  latched_ = *prev_qitr;

  // Remove all elements before latched element
  const auto prev_stamp = prev_qitr->stamp();
  PolicyType::queue_.remove_before(prev_stamp);
  return State::PRIMED;
}

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Latched<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  // don't remove anything abort
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Latched<DispatchT, LockPolicyT, AllocatorT>::reset_follower_impl() noexcept(true)
{
  latched_.reset();
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_LATCHED_HPP
