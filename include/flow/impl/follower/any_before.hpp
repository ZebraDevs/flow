/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_ANY_BEFORE_HPP
#define FLOW_IMPL_FOLLOWER_ANY_BEFORE_HPP

// C++ Standard Library
#include <cstdint>
#include <mutex>
#include <stdexcept>


namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::AnyBefore(
  const offset_type& delay,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor},
    delay_{delay}
{}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
std::tuple<State, ExtractionRange> AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.upper_stamp - delay_;

  auto itr = PolicyType::queue_.begin();

  // Collect all the messages that are earlier than the first driving message's
  // timestamp minus the delay
  while (itr != PolicyType::queue_.end() and get_stamp(*itr) < boundary)
  {
    ++itr;
  }

  return std::make_tuple(
    State::PRIMED, ExtractionRange{0, static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), itr))});
}

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
void AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::extract_follower_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  output = PolicyType::queue_.move(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.last);
}

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
void AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort - delay_);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_ANY_BEFORE_HPP
