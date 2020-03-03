/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_FOLLOWER_IMPL_COUNT_HPP
#define FLOW_FOLLOWER_IMPL_COUNT_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace flow
{
namespace follower
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Count<DispatchT, LockPolicyT, AllocatorT>::Count(const offset_type& delay,
                                                 const size_type n_before,
                                                 const size_type m_after) :
  PolicyType{},
  delay_{delay},
  n_before_{n_before},
  m_after_{m_after}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Count<DispatchT, LockPolicyT, AllocatorT>::Count(const offset_type& delay,
                                                 const size_type n_before,
                                                 const size_type m_after,
                                                 const AllocatorT& alloc) :
  PolicyType{alloc},
  delay_{delay},
  n_before_{n_before},
  m_after_{m_after}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Count<DispatchT, LockPolicyT, AllocatorT>::capture_follower_impl(OutputDispatchIteratorT&& output,
                                                                       const CaptureRange<stamp_type>& range)
{
  if (PolicyType::queue_.empty())
  {
    // Abort early on empty queue
    return State::RETRY;
  }

  // Find initial start position iterator
  auto first = PolicyType::queue_.seek_before(range.lower_stamp - delay_, PolicyType::queue_.begin());

  // Abort if there are no messages before
  if (first == PolicyType::queue_.end())
  {
    return State::ABORT;
  }

  // Step first iterator backwards by n_before_ (first will always start 1 before range)
  {
    size_type remaining = n_before_;
    while (remaining and first != PolicyType::queue_.begin())
    {
      --first;
      --remaining;
    }

    // Must abort if could not rewind far enough
    if (remaining)
    {
      return State::ABORT;
    }
  }

  // Find initial end position iterator
  auto last = PolicyType::queue_.seek_after(range.upper_stamp - delay_, first);

  // Step last iterator forward by m_after_ - 1  (m_after_ > 0, always)
  {
    size_type remaining = m_after_ - 1;
    while (remaining and last != PolicyType::queue_.end())
    {
      ++last;
      --remaining;
    }

    // Must retry if could not increment far enough
    if (remaining)
    {
      return State::RETRY;
    }
  }

  // Copy captured data over range
  std::copy(first, last, output);

  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Count<DispatchT, LockPolicyT, AllocatorT>::abort_follower_impl(const stamp_type& t_abort)
{
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
inline void Count<DispatchT, LockPolicyT, AllocatorT>::validate() const
{
  if (m_after_ < 1)
  {
    throw std::invalid_argument{"m_after_ must be > 0; m_after_ == 0 could cause for non-deterministic synchronization"};
  }
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_FOLLOWER_IMPL_COUNT_HPP
