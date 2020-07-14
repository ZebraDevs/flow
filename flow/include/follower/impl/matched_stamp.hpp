/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_MATCHED_STAMP_HPP
#define FLOW_FOLLOWER_IMPL_MATCHED_STAMP_HPP

// C++ Standard Library
#include <iterator>

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
MatchedStamp<DispatchT, LockPolicyT, AllocatorT>::MatchedStamp(const AllocatorT& alloc) : PolicyType{alloc}
{}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
template <typename OutputDispatchIteratorT>
State MatchedStamp<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(
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
State MatchedStamp<DispatchT, LockPolicyT, AllocatorT>::dry_capture_follower_impl(const CaptureRange<stamp_type>& range)
{
  // Remove all elements before leading time
  PolicyType::queue_.remove_before(range.lower_stamp);

  if (PolicyType::queue_.empty())
  {
    return State::RETRY;
  }
  else if (PolicyType::queue_.oldest_stamp() > range.upper_stamp)
  {
    return State::ABORT;
  }

  // Assign matching element
  return State::PRIMED;
}


template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
void MatchedStamp<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_MATCHED_STAMP_HPP
