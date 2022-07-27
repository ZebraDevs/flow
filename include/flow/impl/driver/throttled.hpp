/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_DRIVER_TRHOTTLED_HPP
#define FLOW_IMPL_DRIVER_TRHOTTLED_HPP

// C++ Standard Library
#include <iterator>

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
Throttled<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::Throttled(
  const offset_type throttle_period,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) noexcept(false) :
    PolicyType{container, queue_monitor},
    throttle_period_{throttle_period},
    previous_stamp_{StampTraits<stamp_type>::min()}
{}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
std::tuple<State, ExtractionRange>
Throttled<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_driver_impl(
  CaptureRange<stamp_type>& range) const
{
  std::size_t index = 0UL;
  for (const auto& dispatch : PolicyType::queue_)
  {
    if (
      previous_stamp_ == StampTraits<stamp_type>::min() or
      (AccessStampT::get(dispatch) - previous_stamp_) >= throttle_period_)
    {
      range.lower_stamp = AccessStampT::get(dispatch);
      range.upper_stamp = range.lower_stamp;

      return std::make_tuple(State::PRIMED, ExtractionRange{index, index + 1UL});
    }

    ++index;
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
void Throttled<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::extract_driver_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  if (extraction_range)
  {
    output = PolicyType::queue_.move(output, extraction_range);
    PolicyType::queue_.remove_first_n(extraction_range.last);
    previous_stamp_ = range.lower_stamp;
  }
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Throttled<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_driver_impl(
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
void Throttled<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::reset_driver_impl()
{
  previous_stamp_ = StampTraits<stamp_type>::min();
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_IMPL_DRIVER_TRHOTTLED_HPP
