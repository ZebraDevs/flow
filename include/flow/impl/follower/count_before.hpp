/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_COUNT_BEFORE_HPP
#define FLOW_IMPL_FOLLOWER_COUNT_BEFORE_HPP

// C++ Standard Library
#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::CountBefore(
  const size_type count,
  const offset_type& delay,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor},
    count_{count},
    delay_{delay}
{
  if (count_ == 0)
  {
    throw std::invalid_argument{"'count' cannot be 0"};
  }
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
std::tuple<State, ExtractionRange> CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  // Retry if queue has no data
  if (PolicyType::queue_.empty())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }

  // The boundary before which messages are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.upper_stamp - delay_;

  // Scroll to element boundary
  size_type before_boundary_count = 0;
  auto itr = PolicyType::queue_.begin();
  while (itr != PolicyType::queue_.end() and boundary > get_stamp(*itr))
  {
    ++before_boundary_count;
    ++itr;
  }

  // Count elements after boundary
  const bool at_or_after_boundary_count = itr != PolicyType::queue_.end();

  if (before_boundary_count >= count_)
  {
    const auto first_itr = std::prev(itr, count_);
    return std::make_tuple(
      State::PRIMED,
      ExtractionRange{static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), first_itr)),
                      static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), itr))});
  }
  else if (at_or_after_boundary_count)
  {
    return std::make_tuple(State::ABORT, ExtractionRange{});
  }
  return std::make_tuple(State::RETRY, ExtractionRange{});
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
void CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::extract_follower_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  output = PolicyType::queue_.copy(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.first);
}


}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_COUNT_BEFORE_HPP
