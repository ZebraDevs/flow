/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_SYNCHRONIZER_HPP
#define FLOW_IMPL_SYNCHRONIZER_HPP

// C++ Standard Library
#include <type_traits>
#include <tuple>
#include <utility>

// Flow
#include <flow/impl/apply.hpp>

namespace flow
{
#ifndef DOXYGEN_SKIP
namespace detail
{

/// captor::reset call helper
struct ResetHelper
{
  template<typename CaptorT, typename LockPolicyT>
  inline void operator()(Captor<CaptorT, LockPolicyT>& c)
  {
    c.reset();
  }
};


/// captor::abort call helper
template<typename StampT>
class AbortHelper
{
public:
  explicit AbortHelper(const StampT t_abort) :
    t_abort_{t_abort}
  {}

  template<typename CaptorT, typename LockPolicyT>
  inline void operator()(Captor<CaptorT, LockPolicyT>& c)
  {
    c.abort(t_abort_);
  }
private:
  /// Abort stamp
  StampT t_abort_;
};


/// captor::capture call helper
template<typename ResultT, typename StampT, typename TimePointT>
class CaptureHelper
{
public:
  CaptureHelper(ResultT& result,
                const StampT& t_latest,
                const TimePointT timeout) :
    result_{std::addressof(result)},
    t_latest_{t_latest},
    timeout_{timeout}
  {}

  template<typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state
    result_->state = c.capture(output, result_->range);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < t_latest_)
    {
      result_->state = State::ABORT;
    }
  }

  template<typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<is_polling<PolicyT>::value> operator()(Follower<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      result_->state = c.capture(output, result_->range);
    }
  }

  template<typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<!is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state
    result_->state = c.capture(output, result_->range, timeout_);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < t_latest_)
    {
      result_->state = State::ABORT;
    }
  }

  template<typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<!is_polling<PolicyT>::value> operator()(Follower<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      result_->state = c.capture(output, result_->range, timeout_);
    }
  }

private:
  /// Capture result
  ResultT* const result_;

  /// Known latest sequence stamp
  StampT t_latest_;

  /// Time at which data waits should end
  TimePointT timeout_;
};



/// captor::dry_capture call helper
template<typename ResultT, typename StampT>
class DryCaptureHelper
{
public:
  DryCaptureHelper(ResultT& result, const StampT& t_latest) :
    result_{std::addressof(result)},
    t_latest_{t_latest}
  {}

  template<typename PolicyT>
  inline void operator()(Driver<PolicyT>& c)
  {
    // Get capture state
    result_->state = c.dry_capture(result_->range);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < t_latest_)
    {
      result_->state = State::ABORT;
    }
  }

  template<typename PolicyT>
  inline void operator()(Follower<PolicyT>& c)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      result_->state = c.dry_capture(result_->range);
    }
  }

private:
  /// Capture result
  ResultT* const result_;

  /// Known latest sequence stamp
  StampT t_latest_;
};

}  // namespace detail
#endif  // DOXYGEN_SKIP


template<typename... CaptorTs>
Synchronizer<CaptorTs...>::Synchronizer(const stamp_type latest_stamp) :
  latest_stamp_{latest_stamp}
{
  // Make sure that 'stamp_type's for all captors are the same
  detail::check_stamp_type<CaptorTs...>();
}


template<typename... CaptorTs>
Synchronizer<CaptorTs...>::~Synchronizer()
{
}


template<typename... CaptorTs>
template<typename ClockT, typename DurationT, typename... OutputIteratorTs>
typename
Synchronizer<CaptorTs...>::Result
Synchronizer<CaptorTs...>::capture(const std::tuple<CaptorTs&...>& captors,
                                   const std::tuple<OutputIteratorTs...> outputs,
                                   const std::chrono::time_point<ClockT, DurationT> timeout)
{
  using time_point_type = std::chrono::time_point<ClockT, DurationT>;

  // Capture next input set
  Result result;
  apply_every(detail::CaptureHelper<Result, stamp_type, time_point_type>{result, latest_stamp_, timeout},
              captors,
              outputs);

  // Update sequence monotonicity guard
  if (result.range and result.state != State::RETRY)
  {
    latest_stamp_ = std::max(result.range.lower_stamp, latest_stamp_);
  }
  return result;
}


template<typename... CaptorTs>
template<typename CaptorTupleT, typename OutputIteratorTupleT>
typename
Synchronizer<CaptorTs...>::Result
Synchronizer<CaptorTs...>::capture(CaptorTupleT&& captors, OutputIteratorTupleT&& outputs)
{
  return this->capture(std::forward<CaptorTupleT>(captors),
                       std::forward<OutputIteratorTupleT>(outputs),
                       std::chrono::steady_clock::time_point::max());
}


template<typename... CaptorTs>
typename
Synchronizer<CaptorTs...>::Result
Synchronizer<CaptorTs...>::dry_capture(const std::tuple<CaptorTs&...>& captors) const
{
  // Capture next input set
  Result result;
  apply_every(detail::DryCaptureHelper<Result, stamp_type>{result, latest_stamp_}, captors);

  return result;
}


template<typename... CaptorTs>
void Synchronizer<CaptorTs...>::abort(const std::tuple<CaptorTs&...>& captors, const stamp_type t_abort)
{
  apply_every(detail::AbortHelper<stamp_type>{t_abort}, captors);
}


template<typename... CaptorTs>
void Synchronizer<CaptorTs...>::reset(const std::tuple<CaptorTs&...>& captors)
{
  latest_stamp_ = StampTraits<stamp_type>::min();
  apply_every(detail::ResetHelper{}, captors);
}

}  // namespace flow

#endif  // FLOW_IMPL_SYNCHRONIZER_HPP
