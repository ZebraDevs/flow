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
template<typename ResultT, typename StampT>
class CaptureHelper
{
public:
  CaptureHelper(ResultT& result,
                const StampT& t_latest,
                const std::chrono::system_clock::time_point timeout) :
    result_{std::addressof(result)},
    t_latest_{t_latest},
    timeout_{timeout}
  {}

  /// Abort caller for driving captors
  template<typename PolicyT, typename OutputIteratorT>
  inline void operator()(Driver<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state
    result_->state = c.capture(output, result_->range, timeout_);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->range.upper_stamp < t_latest_)
    {
      result_->state = State::ABORT;
    }
  }

  /// Abort caller for follower captors
  template<typename PolicyT, typename OutputIteratorT>
  inline void operator()(Follower<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      result_->state = c.capture(output, result_->range, timeout_);
    }
  }

private:
  /// Capture result
  ResultT* result_;

  /// Known latest sequence stamp
  StampT t_latest_;

  /// System time to end data waits
  std::chrono::system_clock::time_point timeout_;
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
template<typename... OutputIteratorTs>
typename
Synchronizer<CaptorTs...>::Result
Synchronizer<CaptorTs...>::capture(const std::tuple<CaptorTs&...>& captors,
                                   const std::tuple<OutputIteratorTs...> outputs,
                                   const std::chrono::system_clock::time_point timeout)
{
  // Capture next input set
  Result result;
  apply_every(detail::CaptureHelper<Result, stamp_type>{result, latest_stamp_, timeout},
              captors,
              outputs);

  // Update sequence monotonicity guard
  latest_stamp_ = std::max(result.range.lower_stamp, latest_stamp_);

  // Check for aborted capture
  if (result.state == State::ABORT)
  {
    abort(captors, latest_stamp_);
  }
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
