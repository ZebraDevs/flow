/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_RANGED_HPP
#define FLOW_IMPL_FOLLOWER_RANGED_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

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
Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::Ranged(
  const offset_type& delay,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor},
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
Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  // Abort early on empty queue
  if (PolicyType::queue_.empty())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }

  // Get iterator to element after first in capture sequence
  const auto after_first_itr = this->find_after_first(range);

  // If we are at the start of the available range, then all elements after this one will be after the valid range
  if (after_first_itr == PolicyType::queue_.begin())
  {
    return std::make_tuple(State::ABORT, ExtractionRange{});
  }

  // Find initial end position iterator
  const auto before_last_itr = this->find_before_last(range, after_first_itr);

  // If the end of our range
  if (before_last_itr == PolicyType::queue_.end())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }

  // Get capture iterator range
  const auto first_itr = std::prev(after_first_itr);
  const auto last_itr = std::next(before_last_itr);

  return std::make_tuple(
    State::PRIMED,
    ExtractionRange{static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), first_itr)),
                    static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), last_itr))});
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
template <typename OutputDispatchIteratorT>
void Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::extract_follower_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  output = PolicyType::queue_.copy(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.first);
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
auto Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::find_after_first(
  const CaptureRange<stamp_type>& range) const
{
  return std::find_if(
    PolicyType::queue_.begin(),
    PolicyType::queue_.end(),
    [offset_lower_stamp = (range.lower_stamp - delay_)](const DispatchT& dispatch) {
      return AccessStampT::get(dispatch) >= offset_lower_stamp;
    });
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
template <typename QueueIteratorT>
auto Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::find_before_last(
  const CaptureRange<stamp_type>& range,
  const QueueIteratorT after_first) const
{
  return std::find_if_not(
    after_first == PolicyType::queue_.end() ? PolicyType::queue_.begin() : after_first,
    PolicyType::queue_.end(),
    [offset_upper_stamp = (range.upper_stamp - delay_)](const DispatchT& dispatch) {
      return AccessStampT::get(dispatch) <= offset_upper_stamp;
    });
}

template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_follower_impl(
  const stamp_type& t_abort)
{}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_RANGED_HPP
