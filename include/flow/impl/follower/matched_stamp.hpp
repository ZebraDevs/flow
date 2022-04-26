/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_MATCHED_STAMP_HPP
#define FLOW_IMPL_FOLLOWER_MATCHED_STAMP_HPP

// C++ Standard Library
#include <iterator>

namespace flow
{
namespace follower
{

template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
MatchedStamp<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::MatchedStamp(
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor}
{}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
State MatchedStamp<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::capture_follower_impl(
  OutputDispatchIteratorT output,
  const CaptureRange<stamp_type>& range)
{
  const auto locate_result = MatchedStamp::locate_follower_impl(range);
  MatchedStamp::extract_follower_impl(output, std::get<1>(locate_result), range);
  return std::get<0>(locate_result);
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
std::tuple<State, ExtractionRange>
MatchedStamp<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.empty())
  {
    return std::make_tuple(State::RETRY, ExtractionRange{});
  }
  else if (PolicyType::queue_.oldest_stamp() > range.upper_stamp)
  {
    return std::make_tuple(State::ABORT, ExtractionRange{});
  }

  ExtractionRange extraction_range;

  std::size_t element_index = 0;
  for (auto element_itr = PolicyType::queue_.begin(); element_itr != PolicyType::queue_.end();
       ++element_itr, ++element_index)
  {
    if (get_stamp(*element_itr) < range.lower_stamp or get_stamp(*element_itr) > range.upper_stamp)
    {
      continue;
    }
    else if (!extraction_range)
    {
      extraction_range.first = element_index;
      extraction_range.last = element_index + 1UL;  // one-past last (iterator-like)
    }
    else
    {
      ++extraction_range.last;
    }
  }

  // Assign matching element
  return std::make_tuple(static_cast<bool>(extraction_range) ? State::PRIMED : State::RETRY, extraction_range);
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT>
void MatchedStamp<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::extract_follower_impl(
  OutputDispatchIteratorT output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  PolicyType::queue_.copy(output, extraction_range);
  PolicyType::queue_.remove_first_n(extraction_range.first);
}


template <typename DispatchT, typename LockPolicyT, typename ContainerT, typename QueueMonitorT>
void MatchedStamp<DispatchT, LockPolicyT, ContainerT, QueueMonitorT>::abort_follower_impl(const stamp_type& t_abort)
{
  PolicyType::queue_.remove_before(t_abort);
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_MATCHED_STAMP_HPP
