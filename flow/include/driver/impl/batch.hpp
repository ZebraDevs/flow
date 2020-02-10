/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_DRIVER_IMPL_BATCH_HPP
#define FLOW_DRIVER_IMPL_BATCH_HPP

// C++ Standard Library
#include <stdexcept>

namespace flow
{
namespace driver
{

template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Batch<DispatchT, LockPolicyT, AllocatorT>::Batch(size_type size) noexcept(false) :
  batch_size_{size}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
Batch<DispatchT, LockPolicyT, AllocatorT>::Batch(size_type size, const AllocatorT& alloc) noexcept(false) :
  PolicyType{alloc},
  batch_size_{size}
{
  validate();
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
State Batch<DispatchT, LockPolicyT, AllocatorT>::capture_driver_impl(OutputDispatchIteratorT output,
                                                                     CaptureRange<stamp_type>& range)
{
  // Continue waiting for data if queue does not yet contain enough elements
  if (PolicyType::queue_.size() < batch_size_)
  {
    return State::RETRY;
  }

  // Collect dispatches
  range.lower_stamp = PolicyType::queue_.oldest_stamp();

  size_type count = 0;
  for (auto& d : PolicyType::queue_)
  {
    // Abort when batch has been captured
    if (count == batch_size_)
    {
      break;
    }

    ++count;

    // Store data to output
    *(output++) = d;

    // Get latest stamp
    range.upper_stamp = d.stamp();
  }

  // Pop last element
  PolicyType::queue_.pop();
  return State::PRIMED;
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Batch<DispatchT, LockPolicyT, AllocatorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}


template<typename DispatchT, typename LockPolicyT, typename AllocatorT>
void Batch<DispatchT, LockPolicyT, AllocatorT>::validate() const noexcept(false)
{
  if(batch_size_ == 0)
  {
    throw std::invalid_argument{"'batch_size_' should be greater than zero"};
  }
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_DRIVER_IMPL_BATCH_HPP
