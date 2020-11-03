/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP
#define FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>

namespace flow
{

template <typename CaptorT, typename LockableT, typename QueueMonitorT>
Captor<CaptorT, LockableT, QueueMonitorT>::Captor() :
    CaptorInterfaceType{0UL, DispatchContainerType{}, QueueMonitorT{}},
    capturing_{true}
{}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
Captor<CaptorT, LockableT, QueueMonitorT>::Captor(
  const DispatchContainerType& container,
  const QueueMonitorT& queue_monitor) :
    CaptorInterfaceType{0UL, container, queue_monitor},
    capturing_{true}
{}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
Captor<CaptorT, LockableT, QueueMonitorT>::~Captor()
{
  abort_impl(StampTraits<stamp_type>::max());
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
typename Captor<CaptorT, LockableT, QueueMonitorT>::size_type
Captor<CaptorT, LockableT, QueueMonitorT>::size_impl() const
{
  LockableT lock{capture_mutex_};
  return CaptorInterfaceType::queue_.size();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
void Captor<CaptorT, LockableT, QueueMonitorT>::reset_impl()
{
  {
    LockableT lock{capture_mutex_};

    // Indicate that capture should stop
    capturing_ = false;

    // Run reset behavior specific to this captor
    derived()->reset_policy_impl();

    // Remove all data
    CaptorInterfaceType::queue_.clear();
  }

  // Release capture waits
  capture_cv_.notify_one();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
void Captor<CaptorT, LockableT, QueueMonitorT>::abort_impl(const stamp_type& t_abort)
{
  {
    LockableT lock{capture_mutex_};

    // Indicate that capture should stop
    capturing_ = false;

    // Run abort behavior specific to this captor
    derived()->abort_policy_impl(t_abort);
  }

  // Release capture waits
  capture_cv_.notify_one();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
template <typename... DispatchConstructorArgTs>
void Captor<CaptorT, LockableT, QueueMonitorT>::inject_impl(DispatchConstructorArgTs&&... dispatch_args)
{
  {
    // Insert new data
    LockableT lock{capture_mutex_};
    CaptorInterfaceType::insert_and_limit(std::forward<DispatchConstructorArgTs>(dispatch_args)...);
  }

  // Notify that new data has arrived
  capture_cv_.notify_one();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
template <typename FirstForwardDispatchIteratorT, typename LastForwardDispatchIteratorT>
void Captor<CaptorT, LockableT, QueueMonitorT>::insert_impl(
  FirstForwardDispatchIteratorT first,
  LastForwardDispatchIteratorT last)
{
  {
    // Insert new data
    LockableT lock{capture_mutex_};
    std::for_each(
      first, last, [this](const DispatchType& dispatch) { CaptorInterfaceType::insert_and_limit(dispatch); });
  }

  // Notify that new data has arrived
  capture_cv_.notify_one();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
void Captor<CaptorT, LockableT, QueueMonitorT>::remove_impl(const stamp_type& t_remove)
{
  {
    // Remove all data before this time
    LockableT lock{capture_mutex_};
    CaptorInterfaceType::queue_.remove_before(t_remove);
  }

  // Notify that data has changed
  capture_cv_.notify_one();
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
void Captor<CaptorT, LockableT, QueueMonitorT>::set_capacity_impl(size_type capacity)
{
  LockableT lock{capture_mutex_};
  CaptorInterfaceType::capacity_ = capacity;
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
typename Captor<CaptorT, LockableT, QueueMonitorT>::size_type
Captor<CaptorT, LockableT, QueueMonitorT>::get_capacity_impl() const
{
  LockableT lock{capture_mutex_};
  return CaptorInterfaceType::capacity_;
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
CaptureRange<typename Captor<CaptorT, LockableT, QueueMonitorT>::stamp_type>
Captor<CaptorT, LockableT, QueueMonitorT>::get_available_stamp_range_impl() const
{
  LockableT lock{capture_mutex_};
  return queue_.empty() ? CaptureRange<stamp_type>{}
                        : CaptureRange<stamp_type>{queue_.oldest_stamp(), queue_.newest_stamp()};
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
template <typename OutputDispatchIteratorT, typename CaptureRangeT, typename ClockT, typename DurationT>
State Captor<CaptorT, LockableT, QueueMonitorT>::capture_impl(
  OutputDispatchIteratorT&& output,
  CaptureRangeT&& range,
  const std::chrono::time_point<ClockT, DurationT> timeout)
{
  LockableT lock{capture_mutex_};

  // Wait for data and attempt capture when data is available
  State state{State::ABORT};
  while (capturing_)
  {
    // Attempt data capture
    state =
      derived()->capture_policy_impl(std::forward<OutputDispatchIteratorT>(output), std::forward<CaptureRangeT>(range));

    // Check capture state, and whether or not a data wait is needed
    if (state != State::RETRY)
    {
      break;
    }
    else if (std::chrono::time_point<ClockT, DurationT>::max() == timeout)
    {
      capture_cv_.wait(lock);
    }
    else if (std::cv_status::timeout == capture_cv_.wait_until(lock, timeout))
    {
      state = State::TIMEOUT;
      break;
    }
  }

  if (capturing_)
  {
    // Return state set in capture loop if not aborted externally
    return state;
  }
  else
  {
    // Make sure capturing flag is reset under lock before next capture attempt
    capturing_ = true;
    return (state == State::RETRY) ? State::ABORT : state;
  }
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
template <typename CaptureRangeT>
State Captor<CaptorT, LockableT, QueueMonitorT>::dry_capture_impl(CaptureRangeT&& range)
{
  LockableT lock{capture_mutex_};

  return derived()->dry_capture_policy_impl(std::forward<CaptureRangeT>(range));
}


template <typename CaptorT, typename LockableT, typename QueueMonitorT>
template <typename InpectCallbackT>
void Captor<CaptorT, LockableT, QueueMonitorT>::inspect_impl(InpectCallbackT&& inspect_dispatch_cb) const
{
  LockableT lock{capture_mutex_};

  for (const auto& dispatch : CaptorInterfaceType::queue_)
  {
    inspect_dispatch_cb(dispatch);
  }
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP
