/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_FOLLOWER_LATCHED_HPP
#define FLOW_IMPL_FOLLOWER_LATCHED_HPP

// C++ Standard Library
#include <chrono>

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
Latched<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::Latched(
  const offset_type min_period,
  const ContainerT& container,
  const QueueMonitorT& queue_monitor) :
    PolicyType{container, queue_monitor},
    min_period_{min_period}
{}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
std::tuple<State, ExtractionRange>
Latched<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::locate_follower_impl(
  const CaptureRange<stamp_type>& range) const
{
  if (PolicyType::queue_.empty())
  {
    if (latched_)
    {
      // If we have latched data, return that and report ready state
      return std::make_tuple(State::PRIMED, ExtractionRange{});
    }
    else
    {
      // Retry if queue has no data and there is no latched data
      return std::make_tuple(State::RETRY, ExtractionRange{});
    }
  }

  // The boundary before which data is are valid and after which they are not. Non-inclusive.
  const stamp_type boundary = range.lower_stamp - min_period_;

  // Retry if priming is not possible
  if (PolicyType::queue_.oldest_stamp() > boundary)
  {
    if (latched_)
    {
      // If we have latched data, return that and report ready state
      return std::make_tuple(State::PRIMED, ExtractionRange{});
    }
    else
    {
      // Abort if queue has no data and there is no latched data
      return std::make_tuple(State::ABORT, ExtractionRange{});
    }
  }

  // Find data closest to boundary, starting from oldest
  // curr_qitr will never start at queue_.end() due to previous empty check
  auto curr_qitr = PolicyType::queue_.begin();
  while (curr_qitr != PolicyType::queue_.end() and AccessStampT::get(*curr_qitr) <= boundary)
  {
    ++curr_qitr;
  }
  return std::make_tuple(
    State::PRIMED, ExtractionRange{0, static_cast<std::size_t>(std::distance(PolicyType::queue_.begin(), curr_qitr))});
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
template <typename OutputDispatchIteratorT>
void Latched<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::extract_follower_impl(
  OutputDispatchIteratorT& output,
  const ExtractionRange& extraction_range,
  const CaptureRange<stamp_type>& range)
{
  if (extraction_range)
  {
    const auto access_index = extraction_range.last - 1UL;
    latched_ = *std::next(PolicyType::queue_.begin(), access_index);
    PolicyType::queue_.remove_first_n(access_index);
  }

  if (latched_)
  {
    *(output++) = *latched_;
  }
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Latched<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::abort_follower_impl(
  const stamp_type& t_abort)
{
  // don't remove anything abort
}


template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
void Latched<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>::
  reset_follower_impl() noexcept(true)
{
  latched_.reset();
}

}  // namespace follower
}  // namespace flow

#endif  // FLOW_IMPL_FOLLOWER_LATCHED_HPP
