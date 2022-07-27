/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_DRIVER_BATCH_HPP
#define FLOW_IMPL_DRIVER_BATCH_HPP

// C++ Standard Library
#include <algorithm>
#include <stdexcept>

namespace flow
{
namespace driver
{

template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
Batch<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::Batch(
  const size_type size,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) noexcept(false) :
    PolicyType{container, queue_monitor},
    batch_size_{size}
{
  validate();
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
std::tuple<State, ExtractionRange>
Batch<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_driver_impl(
  CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.size() >= batch_size_)
  {
    auto oldest_itr = PolicyType::queue_.begin();

    range.lower_stamp = AccessStampT::get(*oldest_itr);
    range.upper_stamp = AccessStampT::get(*std::next(oldest_itr, batch_size_ - 1));

    return std::make_tuple(State::PRIMED, ExtractionRange{0, batch_size_});
  }
  return std::make_tuple(State::RETRY, ExtractionRange{});
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
template <typename OutputDispatchIteratorT>
void Batch<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::extract_driver_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  if (extraction_range)
  {
    output = PolicyType::queue_.copy(output, extraction_range);
    PolicyType::queue_.pop();
  }
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Batch<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_driver_impl(
  const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Batch<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::validate() const
  noexcept(false)
{
  if (batch_size_ == 0)
  {
    throw std::invalid_argument{"'batch_size_' should be greater than zero"};
  }
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_IMPL_DRIVER_BATCH_HPP
