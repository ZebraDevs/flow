/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_CLOSEST_BEFORE_HPP
#define FLOW_IMPL_FOLLOWER_CLOSEST_BEFORE_HPP

namespace flow
{
namespace follower
{

template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
ClosestBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::ClosestBefore(
  const offset_type& period,
  const offset_type& delay,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor},
    period_{period},
    delay_{delay}
{}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
std::tuple<State, ExtractionRange>
ClosestBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - delay_;

  // Starting from the oldest data, return on when data found within periodic window
  auto capture_itr = PolicyType::queue_.end();
  for (auto itr = PolicyType::queue_.begin(); itr != PolicyType::queue_.end(); ++itr)
  {
    if (AccessStampT::get(*itr) >= boundary)
    {
      // If oldest element is far ahead of boundary, abort
      return std::make_tuple(State::ABORT, ExtractionRange{});
    }
    else if (AccessStampT::get(*itr) + period_ >= boundary)
    {
      // If oldest element is first within period, collect
      capture_itr = itr;
      break;
    }
  }

  // Check if inputs are capturable
  // NOTE: If no capture input was set, then all data is at or after boundary
  if (capture_itr == PolicyType::queue_.end())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }

  const std::size_t capture_idx = static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), capture_itr));
  return std::make_tuple(State::PRIMED, ExtractionRange{capture_idx, capture_idx + 1UL});
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
template <typename OutputDispatchIteratorT>
void ClosestBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::
  extract_follower_impl(
    OutputDispatchIteratorT& output,
    const ExtractionRange& extraction_range,
    const CaptureRange<stamp_type>& range)
{
  if (extraction_range)
  {
    *(output++) = *std::next(PolicyType::queue_.begin(), extraction_range.first);
    PolicyType::queue_.remove_first_n(extraction_range.first);
  }
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void ClosestBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_follower_impl(
  const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_ - period_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_CLOSEST_BEFORE_HPP
