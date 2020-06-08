/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 * 
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP
#define FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP

// C++ Standard Library
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>

namespace flow
{

template<typename CaptorT, typename LockableT>
Captor<CaptorT, LockableT>::Captor() :
  CaptorInterfaceType{0UL},
  capturing_{true}
{}


template<typename CaptorT, typename LockableT>
Captor<CaptorT, LockableT>::Captor(const DispatchAllocatorType& alloc) :
  CaptorInterfaceType{0UL, alloc},
  capturing_{true}
{}


template<typename CaptorT, typename LockableT>
Captor<CaptorT, LockableT>::~Captor()
{
  abort_impl(StampTraits<stamp_type>::max());
}


template<typename CaptorT, typename LockableT>
typename
Captor<CaptorT, LockableT>::size_type
Captor<CaptorT, LockableT>::size_impl() const
{
  LockableT lock{capture_mutex_};
  return CaptorInterfaceType::queue_.size();
}


template<typename CaptorT, typename LockableT>
void Captor<CaptorT, LockableT>::reset_impl()
{
  LockableT lock{capture_mutex_};
  derived()->reset_policy_impl();
  CaptorInterfaceType::queue_.clear();
}


template<typename CaptorT, typename LockableT>
void Captor<CaptorT, LockableT>::abort_impl(const stamp_type& t_abort)
{
  {
    LockableT lock{capture_mutex_};

    // Indicate that capture should stop
    capturing_ = false;

    // Run abort behavior for this captor
    derived()->abort_policy_impl(t_abort);
  }

  // Release capture waits
  capture_cv_.notify_one();
}


template<typename CaptorT, typename LockableT>
template<typename... DispatchConstructorArgTs>
void Captor<CaptorT, LockableT>::inject_impl(DispatchConstructorArgTs&&... dispatch_args)
{
  {
    // Insert new data
    LockableT lock{capture_mutex_};
    CaptorInterfaceType::insert_and_limit(std::forward<DispatchConstructorArgTs>(dispatch_args)...);
  }

  // Notify that new data has arrived
  capture_cv_.notify_one();
}


template<typename CaptorT, typename LockableT>
void Captor<CaptorT, LockableT>::set_capacity_impl(size_type capacity)
{
  LockableT lock{capture_mutex_};
  CaptorInterfaceType::capacity_ = capacity;
}


template<typename CaptorT, typename LockableT>
typename
Captor<CaptorT, LockableT>::size_type
Captor<CaptorT, LockableT>::get_capacity_impl() const
{
  LockableT lock{capture_mutex_};
  return CaptorInterfaceType::capacity_;
}


template<typename CaptorT, typename LockableT>
CaptureRange<typename Captor<CaptorT, LockableT>::stamp_type>
Captor<CaptorT, LockableT>::get_available_stamp_range_impl() const
{
  LockableT lock{capture_mutex_};
  return queue_.empty() ?
         CaptureRange<stamp_type>{} :
         CaptureRange<stamp_type>{queue_.oldest_stamp(), queue_.newest_stamp()};
}


template<typename CaptorT, typename LockableT>
template<typename OutputDispatchIteratorT, typename CaptureRangeT, typename ClockT, typename DurationT>
State Captor<CaptorT, LockableT>::capture_impl(OutputDispatchIteratorT&& output,
                                               CaptureRangeT&& range,
                                               const std::chrono::time_point<ClockT, DurationT> timeout)
{
  LockableT lock{capture_mutex_};

  // Reset flag if cancelled externally on previous capture
  capturing_ = true;

  // Wait for data and attempt capture when data is available
  do
  {
    // Attempt data capture
    const State state = derived()->capture_policy_impl(std::forward<OutputDispatchIteratorT>(output),
                                                       std::forward<CaptureRangeT>(range));

    // Check capture state, and whether or not a data wait is needed
    if (state != State::RETRY)
    {
      return state;
    }
    else if (std::chrono::time_point<ClockT, DurationT>::max() == timeout)
    {
      capture_cv_.wait(lock);
    }
    else if (std::cv_status::timeout == capture_cv_.wait_until(lock, timeout))
    {
      return State::TIMEOUT;
    }
  }
  while (capturing_);

  // Make sure capturing flag is reset under lock before next capture attempt
  return State::ABORT;
}


template<typename CaptorT, typename LockableT>
template<typename InpectCallbackT>
void Captor<CaptorT, LockableT>::inspect_impl(InpectCallbackT&& inspect_dispatch_cb) const
{
  LockableT lock{capture_mutex_};

  for (const auto& dispatch : CaptorInterfaceType::queue_)
  {
    inspect_dispatch_cb(dispatch);
  }
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_LOCKABLE_HPP
