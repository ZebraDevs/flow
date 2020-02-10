/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_COUNT_HPP
#define FLOW_FOLLOWER_IMPL_COUNT_HPP

// C++ Standard Library
#include <iterator>


namespace flow
{
namespace follower
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Count<DispatchT, LockPolicyT, AllocatorT>::Count(const size_type n_before,
                                                 const size_type m_after) :
  PolicyType{},
  n_before_{n_before},
  m_after_{m_after}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Count<DispatchT, LockPolicyT, AllocatorT>::Count(const size_type n_before,
                                                 const size_type m_after,
                                                 const AllocatorT& alloc) :
  PolicyType{alloc},
  n_before_{n_before},
  m_after_{m_after}
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Count<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT output,
                                                                       const CaptureRange<stamp_type>& range)
{
  if (PolicyType::queue_.empty())
  {
    // Abort early on empty queue
    return State::RETRY;
  }

  // Find messages around sequence stamp targets
  const auto counts = PolicyType::queue_.capture_around(output,
                                                        range,
                                                        n_before_,
                                                        m_after_);
  if (std::get<0>(counts) < n_before_)
  {
    // Abort if there are no messages before
    return State::ABORT;
  }
  else if (std::get<1>(counts) < m_after_)
  {
    // Wait for messages after target sequence stamp range
    return State::RETRY;
  }
  else if (std::get<1>(counts) >= m_after_)
  {
    // Capture when enough data is available
    PolicyType::queue_.remove_before(std::get<2>(counts));
    return State::PRIMED;
  }
  return State::RETRY;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Count<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_COUNT_HPP
