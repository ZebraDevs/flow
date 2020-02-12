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

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Latched<DispatchT, LockPolicyT, AllocatorT>::Latched(const offset_type min_period) :
  PolicyType{},
  latched_{nullptr},
  min_period_{min_period}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Latched<DispatchT, LockPolicyT, AllocatorT>::Latched(const offset_type min_period, const AllocatorT& alloc) :
  PolicyType{alloc},
  latched_{nullptr},
  min_period_{min_period}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Latched<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT output,
                                                                         const CaptureRange<stamp_type>& range)
{
  if (PolicyType::queue_.empty())
  {
    if (latched_ == nullptr)
    {
      // Retry if queue has no data and there is no latched data
      return State::RETRY;
    }
    else
    {
      // If we have latched data, return that and report ready state
      *(output++) = *latched_;
      return State::PRIMED;
    }
  }

  // The boundary before which data is are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - min_period_;

  // Retry if priming is not possible
  if (PolicyType::queue_.oldest_stamp() > boundary)
  {
    if (latched_ == nullptr)
    {
      // Abort if queue has no data and there is no latched data
      return State::ABORT;
    }
    else
    {
      // If we have latched data, return that and report ready state
      *(output++) = *latched_;
      return State::PRIMED;
    }
  }

  // Find data closest to boundary, starting from oldest
  auto curr_qitr = PolicyType::queue_.begin();
  auto prev_qitr = curr_qitr;
  while (curr_qitr != PolicyType::queue_.end() and curr_qitr->stamp() <= boundary)
  {
    prev_qitr = curr_qitr;
    ++curr_qitr;
  }

  // Set latched element
  latched_ = std::addressof(*prev_qitr);
  *(output++) = *latched_;

  // Remove all elements before latched element
  PolicyType::queue_.remove_before(latched_->stamp());
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Latched<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  // don't remove anything abort
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Latched<DispatchT, LockPolicyT, AllocatorT>::reset_follower_impl() noexcept(true)
{
  latched_ = nullptr;
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_LATCHED_HPP
