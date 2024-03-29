/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_DRIVER_CHUNK_HPP
#define FLOW_IMPL_DRIVER_CHUNK_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
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
Chunk<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::Chunk(
  const size_type size,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) noexcept(false) :
    PolicyType{container, queue_monitor},
    chunk_size_{size}
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
Chunk<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_driver_impl(
  CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.size() >= chunk_size_)
  {
    auto oldest_itr = PolicyType::queue_.begin();

    range.lower_stamp = AccessStampT::get(*oldest_itr);
    range.upper_stamp = AccessStampT::get(*std::next(oldest_itr, chunk_size_ - 1));

    return std::make_tuple(State::PRIMED, ExtractionRange{0, chunk_size_});
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
void Chunk<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::extract_driver_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  output = PolicyType::queue_.move(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.last);
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Chunk<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_driver_impl(
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
void Chunk<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::validate() const
  noexcept(false)
{
  if (chunk_size_ == 0)
  {
    throw std::invalid_argument{"'chunk_size_' should be greater than zero"};
  }
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_IMPL_DRIVER_CHUNK_HPP
