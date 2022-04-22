/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_DRIVER_NEXT_HPP
#define FLOW_IMPL_DRIVER_NEXT_HPP

namespace flow
{
namespace driver
{

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::Next(
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) noexcept(false) :
    PolicyType{container, queue_monitor}
{}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
State Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::capture_driver_impl(
  OutputDispatchIteratorT output,
  CaptureRange<stamp_type>& range)
{
  const auto locate_result = Next::locate_driver_impl(range);
  Next::extract_driver_impl(output, std::get<1>(locate_result), range);
  return std::get<0>(locate_result);
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
std::tuple<State, ExtractionRange>
Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::locate_driver_impl(CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.empty())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }

  range.lower_stamp = PolicyType::queue_.oldest_stamp();
  range.upper_stamp = range.lower_stamp;

  return std::make_tuple(State::PRIMED, ExtractionRange{0, 1});
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
void Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::extract_driver_impl(
  OutputDispatchIteratorT output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  PolicyType::queue_.move(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.last);
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
void Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::abort_driver_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace driver
}  // namespace flow

#endif  // FLOW_IMPL_DRIVER_NEXT_HPP
