/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_DRIVER_IMPL_CHUNK_HPP
#define FLOW_DRIVER_IMPL_CHUNK_HPP

// C++ Standard Library
#include <stdexcept>

namespace flow
{
namespace driver
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Chunk<DispatchT, LockPolicyT, AllocatorT>::Chunk(size_type size) noexcept(false) :
  PolicyType{},
  chunk_size_{size}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Chunk<DispatchT, LockPolicyT, AllocatorT>::Chunk(size_type size, const AllocatorT& alloc) noexcept(false) :
  PolicyType{alloc},
  chunk_size_{size}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Chunk<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(OutputDispatchIteratorT output,
                                                                     CaptureRange<stamp_type>& range)
{
  if (PolicyType::queue_.size() < chunk_size_)
  {
    return State::RETRY;
  }

  // Collect dispatches
  range.lower_stamp = PolicyType::queue_.oldest_stamp();

  size_type count = 0;
  while (!PolicyType::queue_.empty())
  {
    // Store data to output; abort when batch has been captured
    if (count == chunk_size_)
    {
      break;
    }

    ++count;

    auto d = PolicyType::queue_.pop();

    // Get latest stamp
    range.upper_stamp = d.stamp();

    // Store data to output
    *(output++) = std::move(d);
  }
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Chunk<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Chunk<DispatchT, LockPolicyT, AllocatorT>::validate() const noexcept(false)
{
  if(chunk_size_ == 0)
  {
    throw std::invalid_argument{"'chunk_size_' should be greater than zero"};
  }
}

}  // namespace driver
}  // namespace flow


#endif  // FLOW_DRIVER_IMPL_CHUNK_HPP
