/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_SYNCHRONIZER_HPP
#define FLOW_IMPL_SYNCHRONIZER_HPP

// C++ Standard Library
#include <tuple>
#include <type_traits>
#include <utility>

// Flow
#include <flow/utility/apply.hpp>
#include <flow/utility/static_assert.hpp>

namespace std
{

/**
 * @brief Specialization of \c std::iterator_traits for a valid \c NoCapture template instance
 */
template <> struct iterator_traits<::flow::NoCapture>
{
  using difference_type = std::ptrdiff_t;
  using value_type = void;
  using pointer = void;
  using reference = void;
  using iterator_category = void;
};

}  // namespace std

namespace flow
{
#ifndef DOXYGEN_SKIP
namespace detail
{

/// helper for use with exchange_type_with
template <typename ExchangeT, typename OriginalT> struct Exchange
{
  using type = ExchangeT;
};

/// creates a new tuple of the same length with all elements of type \c ExchangeT, which must be default-constructible
template <typename ExchangeT, typename... OriginalTs> auto exchange_type_with(std::tuple<OriginalTs...>&)
{
  return std::tuple<typename Exchange<ExchangeT, OriginalTs>::type...>{};
}

/// captor::reset call helper
struct ResetHelper
{
  /// No-op
  template <typename StampT> constexpr void operator()(const CaptureRange<StampT>& range) const {}

  template <typename CaptorT, typename LockPolicyT, typename QueueMonitorT>
  inline void operator()(Captor<CaptorT, LockPolicyT, QueueMonitorT>& c)
  {
    c.reset();
  }
};


/// captor::remove call helper
template <typename StampT> class RemoveHelper
{
public:
  explicit RemoveHelper(const StampT t_remove) : t_remove_{t_remove} {}

  /// No-op
  constexpr void operator()(const CaptureRange<StampT>& range) const {}

  template <typename PolicyT> inline void operator()(Driver<PolicyT>& c) { c.remove(t_remove_); }

  /// No-op
  template <typename PolicyT> constexpr void operator()(Follower<PolicyT>& c) const {}

private:
  /// Remove stamp
  StampT t_remove_;
};


/// captor::abort call helper
template <typename StampT> class AbortHelper
{
public:
  explicit AbortHelper(const StampT t_abort) : t_abort_{t_abort} {}

  /// No-op
  constexpr void operator()(const CaptureRange<StampT>& range) const {}

  template <typename CaptorT, typename LockPolicyT, typename QueueMonitorT>
  inline void operator()(Captor<CaptorT, LockPolicyT, QueueMonitorT>& c)
  {
    c.abort(t_abort_);
  }

private:
  /// Abort stamp
  StampT t_abort_;
};


/// captor::locate call helper
template <typename ResultT, typename StampT, typename TimePointT> class LocateHelper
{
public:
  LocateHelper(ResultT& result, const StampT lower_bound, const TimePointT timeout) :
      result_{std::addressof(result)},
      lower_bound_{lower_bound},
      timeout_{timeout}
  {}

  inline void operator()(const CaptureRange<StampT>& range, ExtractionRange& NO_CAPTURE)
  {
    // Initialize capture range
    result_->range = range;

    // Set aborted state if driving sequence range violates monotonicity guard
    result_->state = range.upper_stamp < lower_bound_ ? State::ABORT : State::PRIMED;
  }

  template <typename PolicyT>
  inline std::enable_if_t<is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, ExtractionRange& extraction_range)
  {
    // Get capture state
    std::tie(result_->state, extraction_range) = c.locate(result_->range);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < lower_bound_)
    {
      result_->state = State::ERROR_DRIVER_LOWER_BOUND_EXCEEDED;
    }
  }

  template <typename PolicyT>
  inline std::enable_if_t<is_polling<PolicyT>::value>
  operator()(Follower<PolicyT>& c, ExtractionRange& extraction_range)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      std::tie(result_->state, extraction_range) = c.locate(result_->range);
    }
  }

  template <typename PolicyT>
  inline std::enable_if_t<!is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, ExtractionRange& extraction_range)
  {
    // Get capture state
    std::tie(result_->state, extraction_range) = c.locate(result_->range, timeout_);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < lower_bound_)
    {
      result_->state = State::ERROR_DRIVER_LOWER_BOUND_EXCEEDED;
    }
  }

  template <typename PolicyT>
  inline std::enable_if_t<!is_polling<PolicyT>::value>
  operator()(Follower<PolicyT>& c, ExtractionRange& extraction_range)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      std::tie(result_->state, extraction_range) = c.locate(result_->range, timeout_);
    }
  }

private:
  /// Capture result
  ResultT* const result_;

  /// Known latest sequence stamp
  StampT lower_bound_;

  /// Time at which data waits should end
  TimePointT timeout_;
};


/// captor::extract call helper
template <typename ResultT> class ExtractHelper
{
public:
  ExtractHelper(ResultT& result) : result_{std::addressof(result)} {}

  template <typename StampT>
  constexpr NoCapture
  operator()(const CaptureRange<StampT>& range, NoCapture __nc__, const ExtractionRange& extraction_range)
  {
    return NoCapture{};
  }

  template <typename CaptorT, typename LockPolicyT, typename QueueMonitorT, typename OutputIteratorT>
  inline OutputIteratorT operator()(
    Captor<CaptorT, LockPolicyT, QueueMonitorT>& c,
    OutputIteratorT output,
    const ExtractionRange& extraction_range)
  {
    output = c.extract(output, extraction_range, result_->range);
    c.update_queue_monitor(result_->range, result_->state);
    return output;
  }

private:
  /// Capture result
  ResultT* const result_;
};

/// Checks that captor stamp types are consistent
template <typename... CaptorTs> struct captor_stamp_types_consistent;

/// Partial specialization for single captor case
template <typename CaptorT> struct captor_stamp_types_consistent<CaptorT> : std::integral_constant<bool, true>
{};

/// Partial specialization for capture range, only, case
template <typename StampT>
struct captor_stamp_types_consistent<CaptureRange<StampT>> : std::integral_constant<bool, true>
{};

/// Partial specialization for drivers + followers case
template <typename Captor1T, typename Captor2T, typename... OtherCaptorTs>
struct captor_stamp_types_consistent<Captor1T, Captor2T, OtherCaptorTs...>
    : std::integral_constant<
        bool,
        std::is_same<
          typename CaptorTraits<std::remove_reference_t<Captor1T>>::stamp_type,
          typename CaptorTraits<std::remove_reference_t<Captor2T>>::stamp_type>::value and
          captor_stamp_types_consistent<OtherCaptorTs...>::value>
{};

/// Partial specialization for direct driving capture range specification
template <typename StampT, typename Captor2T, typename... OtherCaptorTs>
struct captor_stamp_types_consistent<CaptureRange<StampT>, Captor2T, OtherCaptorTs...>
    : std::integral_constant<
        bool,
        std::is_same<StampT, typename CaptorTraits<std::remove_reference_t<Captor2T>>::stamp_type>::value and
          captor_stamp_types_consistent<OtherCaptorTs...>::value>
{};

/// Checks that a sequence of types are all follower captor types (or type sequence is empty)
template <typename... FollowerTs> struct all_captors_are_followers : std::integral_constant<bool, true>
{};

/// Partial specialization for recursive variadic checking
template <typename FollowerT, typename... OtherFollowerTs>
struct all_captors_are_followers<FollowerT, OtherFollowerTs...>
    : std::integral_constant<
        bool,
        is_follower<std::remove_reference_t<FollowerT>>::value and all_captors_are_followers<OtherFollowerTs...>::value>
{};

/// Fall-back case
template <typename CaptorTupleT> struct captor_sequence_valid : std::integral_constant<bool, false>
{};

/// Checks that a sequence of types is (CaptorT, FollowerTs...)
template <template <typename...> class TupleLikeTmpl, typename DriverT, typename... FollowerTs>
struct captor_sequence_valid<TupleLikeTmpl<DriverT, FollowerTs...>>
    : std::integral_constant<
        bool,
        ((is_capture_range<std::remove_const_t<std::remove_reference_t<DriverT>>>::value and
          sizeof...(FollowerTs) > 0) or
         (is_driver<std::remove_reference_t<DriverT>>::value)) and
          all_captors_are_followers<FollowerTs...>::value>
{};

}  // namespace detail
#endif  // DOXYGEN_SKIP


template <typename CaptorTupleT, typename OutputIteratorTupleT, typename ClockT, typename DurationT>
typename std::tuple<Synchronizer::result_t<CaptorTupleT>, OutputIteratorTupleT> Synchronizer::capture(
  CaptorTupleT&& captors,
  OutputIteratorTupleT&& outputs,
  const stamp_arg_t<CaptorTupleT> lower_bound,
  const std::chrono::time_point<ClockT, DurationT>& timeout)
{
  using time_point_type = std::chrono::time_point<ClockT, DurationT>;

  // Sanity check captors and outputs
  constexpr auto N_CAPTORS = std::tuple_size<std::remove_reference_t<CaptorTupleT>>();
  constexpr auto N_OUTPUTS = std::tuple_size<std::remove_reference_t<OutputIteratorTupleT>>();
  FLOW_STATIC_ASSERT(N_OUTPUTS == N_CAPTORS, "[Synchronizer] Number of outputs must match number of captors.");

  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::capture] Captor sequence is invalid. Must have (DriverType, FollowerTypes...) with "
    "0 or more FollowerTypes allowed, or (CaptureRange<StampT>, FollowerTypes...) with at least 1 FollowerTypes.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::capture] Associated captor stamp types do not match between all captors");

  using ResultType = result_t<CaptorTupleT>;
  using StampType = stamp_t<CaptorTupleT>;

  auto elements = detail::exchange_type_with<ExtractionRange>(captors);

  ResultType result;

  // Attempt to locate elements that we will capture
  apply_every(
    detail::LocateHelper<ResultType, StampType, time_point_type>{result, lower_bound, timeout},
    std::forward<CaptorTupleT>(captors),
    elements);

  // If a RETRY state occurs, don't try to capture elements
  if (result.state == State::RETRY)
  {
    return std::make_tuple(result, std::forward<OutputIteratorTupleT>(outputs));
  }

  // Otherwise, capture elements and possibly remove elements from queues
  const auto outputs_advanced = apply_every_r(
    detail::ExtractHelper<ResultType>{result},
    std::forward<CaptorTupleT>(captors),
    std::forward<OutputIteratorTupleT>(outputs),
    elements);

  return std::make_tuple(result, outputs_advanced);
}


template <typename CaptorTupleT, typename OutputIteratorTupleT>
typename std::tuple<Synchronizer::result_t<CaptorTupleT>, OutputIteratorTupleT> Synchronizer::capture(
  CaptorTupleT&& captors,
  OutputIteratorTupleT&& outputs,
  const stamp_arg_t<CaptorTupleT> lower_bound)
{
  return capture(
    std::forward<CaptorTupleT>(captors),
    std::forward<OutputIteratorTupleT>(outputs),
    lower_bound,
    std::chrono::steady_clock::time_point::max());
}

template <typename CaptorTupleT>
void Synchronizer::remove(CaptorTupleT&& captors, const stamp_arg_t<CaptorTupleT> t_remove)
{
  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::remove] Captor sequence is invalid. Must have (DriverType, FollowerTypes...) with "
    "0 or more FollowerTypes allowed, or (CaptureRange<StampT>, FollowerTypes...) with at least 1 FollowerTypes.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::remove] Associated captor stamp types do not match between all captors");

  using StampType = stamp_t<CaptorTupleT>;
  apply_every(detail::RemoveHelper<StampType>{t_remove}, std::forward<CaptorTupleT>(captors));
}


template <typename CaptorTupleT>
void Synchronizer::abort(CaptorTupleT&& captors, const stamp_arg_t<CaptorTupleT> t_abort)
{
  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::abort] Captor sequence is invalid. Must have (DriverType, FollowerTypes...) with "
    "0 or more FollowerTypes allowed, or (CaptureRange<StampT>, FollowerTypes...) with at least 1 FollowerTypes.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::abort] Associated captor stamp types do not match between all captors");

  using StampType = stamp_t<CaptorTupleT>;
  apply_every(detail::AbortHelper<StampType>{t_abort}, std::forward<CaptorTupleT>(captors));
}


template <typename CaptorTupleT> void Synchronizer::reset(CaptorTupleT&& captors)
{
  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::reset] Captor sequence is invalid. Must have (DriverType, FollowerTypes...) with "
    "0 or more FollowerTypes allowed, or (CaptureRange<StampT>, FollowerTypes...) with at least 1 FollowerTypes.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::reset] Associated captor stamp types do not match between all captors");

  apply_every(detail::ResetHelper{}, std::forward<CaptorTupleT>(captors));
}

}  // namespace flow

#endif  // FLOW_IMPL_SYNCHRONIZER_HPP
