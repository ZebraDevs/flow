/**
 * @copyright 2020 Fetch Robotics Inc.
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
#include <flow/impl/apply.hpp>
#include <flow/impl/static_assert.hpp>

namespace flow
{
#ifndef DOXYGEN_SKIP
namespace detail
{

/// captor::reset call helper
struct ResetHelper
{
  template <typename CaptorT, typename LockPolicyT> inline void operator()(Captor<CaptorT, LockPolicyT>& c)
  {
    c.reset();
  }
};

/// captor::remove call helper
template <typename StampT> class RemoveHelper
{
public:
  explicit RemoveHelper(const StampT t_remove) : t_remove_{t_remove} {}

  template <typename PolicyT> inline void operator()(Driver<PolicyT>& c) { c.remove(t_remove_); }

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

  template <typename CaptorT, typename LockPolicyT> inline void operator()(Captor<CaptorT, LockPolicyT>& c)
  {
    c.abort(t_abort_);
  }

private:
  /// Abort stamp
  StampT t_abort_;
};


/// captor::capture call helper
template <typename ResultT, typename StampT, typename TimePointT> class CaptureHelper
{
public:
  CaptureHelper(ResultT& result, const StampT lower_bound, const TimePointT timeout) :
      result_{std::addressof(result)},
      lower_bound_{lower_bound},
      timeout_{timeout}
  {}

  template <typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state
    result_->state = c.capture(output, result_->range);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < lower_bound_)
    {
      result_->state = State::ABORT;
    }
  }

  template <typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<is_polling<PolicyT>::value> operator()(Follower<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state alias
    if (result_->state == State::PRIMED)
    {
      result_->state = c.capture(output, result_->range);
    }
  }

  template <typename PolicyT, typename OutputIteratorT>
  inline std::enable_if_t<!is_polling<PolicyT>::value> operator()(Driver<PolicyT>& c, OutputIteratorT output)
  {
    // Get capture state
    result_->state = c.capture(output, result_->range, timeout_);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < lower_bound_)
    {
      result_->state = State::ABORT;
    }
  }

  template <typename PolicyT, typename OutputIteratorT>
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
  StampT lower_bound_;

  /// Time at which data waits should end
  TimePointT timeout_;
};


/// captor::dry_capture call helper
template <typename ResultT, typename StampT> class DryCaptureHelper
{
public:
  DryCaptureHelper(ResultT& result, const StampT lower_bound) :
      result_{std::addressof(result)},
      lower_bound_{lower_bound}
  {}

  template <typename PolicyT> inline void operator()(Driver<PolicyT>& c)
  {
    // Get capture state
    result_->state = c.dry_capture(result_->range);

    // Set aborted state if driving sequence range violates monotonicity guard
    if (result_->state == State::PRIMED and result_->range.upper_stamp < lower_bound_)
    {
      result_->state = State::ABORT;
    }
  }

  template <typename PolicyT> inline void operator()(Follower<PolicyT>& c)
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
  StampT lower_bound_;
};

/// Checks that captor stamp types are consistnet
template <typename... CaptorTs> struct captor_stamp_types_consistent;

template <typename CaptorT> struct captor_stamp_types_consistent<CaptorT> : std::integral_constant<bool, true>
{};

template <typename Captor1T, typename Captor2T, typename... OtherCaptorTs>
struct captor_stamp_types_consistent<Captor1T, Captor2T, OtherCaptorTs...>
    : std::integral_constant<
        bool,
        std::is_same<
          typename CaptorTraits<std::remove_reference_t<Captor1T>>::stamp_type,
          typename CaptorTraits<std::remove_reference_t<Captor2T>>::stamp_type>::value and
          captor_stamp_types_consistent<OtherCaptorTs...>::value>
{};

/// Checks that a squence of types are all follower captor types (or type sequence is empty)
template <typename... FollowerTs> struct all_captors_are_followers : std::integral_constant<bool, true>
{};

template <typename FollowerT, typename... OtherFollowerTs>
struct all_captors_are_followers<FollowerT, OtherFollowerTs...>
    : std::integral_constant<
        bool,
        is_follower<std::remove_reference_t<FollowerT>>::value and all_captors_are_followers<OtherFollowerTs...>::value>
{};

/// Checks that a squence of types is (CaptorT, FollowerTs...)
template <typename CaptorTupleT> struct captor_sequence_valid : std::integral_constant<bool, false>
{};

template <template <typename...> class TupleLikeTmpl, typename DriverT, typename... FollowerTs>
struct captor_sequence_valid<TupleLikeTmpl<DriverT, FollowerTs...>>
    : std::integral_constant<
        bool,
        is_driver<std::remove_reference_t<DriverT>>::value and all_captors_are_followers<FollowerTs...>::value>
{};

}  // namespace detail
#endif  // DOXYGEN_SKIP


#define FLOW_STATIC_ASSERT_EMPH(msg) "\n\n--->\n\n" msg "\n\n<---\n\n"


template <typename CaptorTupleT, typename OutputIteratorTupleT, typename ClockT, typename DurationT>
typename Synchronizer::result_t<CaptorTupleT> Synchronizer::capture(
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
    "[Synchronizer::capture] Captor sequence is invalid. Must have (DriverType, FollowerTypes...). "
    "0 or more FollowerTypes allowed.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::capture] Associated captor stamp types do not match between all captors");

  using ResultType = result_t<CaptorTupleT>;
  using StampType = stamp_t<CaptorTupleT>;

  ResultType result;
  apply_every(
    detail::CaptureHelper<ResultType, StampType, time_point_type>{result, lower_bound, timeout},
    std::forward<CaptorTupleT>(captors),
    std::forward<OutputIteratorTupleT>(outputs));

  return result;
}


template <typename CaptorTupleT, typename OutputIteratorTupleT>
typename Synchronizer::result_t<CaptorTupleT> Synchronizer::capture(
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
typename Synchronizer::result_t<CaptorTupleT>
Synchronizer::dry_capture(CaptorTupleT&& captors, const stamp_arg_t<CaptorTupleT> lower_bound)
{
  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::dry_capture] Captor sequence is invalid. Must have (DriverType, FollowerTypes...). "
    "0 or more FollowerTypes allowed.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::dry_capture] Associated captor stamp types do not match between all captors");

  using ResultType = result_t<CaptorTupleT>;
  using StampType = stamp_t<CaptorTupleT>;

  // Capture next input set
  ResultType result;
  apply_every(
    detail::DryCaptureHelper<ResultType, StampType>{result, lower_bound}, std::forward<CaptorTupleT>(captors));

  return result;
}


template <typename CaptorTupleT>
void Synchronizer::remove(CaptorTupleT&& captors, const stamp_arg_t<CaptorTupleT> t_remove)
{
  // Sanity check captor sequence
  FLOW_STATIC_ASSERT(
    detail::captor_sequence_valid<CaptorTupleT>(),
    "[Synchronizer::remove] Captor sequence is invalid. Must have (DriverType, FollowerTypes...). "
    "0 or more FollowerTypes allowed.");

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
    "[Synchronizer::abort] Captor sequence is invalid. Must have (DriverType, FollowerTypes...). "
    "0 or more FollowerTypes allowed.");

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
    "[Synchronizer::reset] Captor sequence is invalid. Must have (DriverType, FollowerTypes...). "
    "0 or more FollowerTypes allowed.");

  // Sanity check captor stamp types
  FLOW_STATIC_ASSERT(
    detail::captor_stamp_types_consistent<CaptorTupleT>(),
    "[Synchronizer::reset] Associated captor stamp types do not match between all captors");

  apply_every(detail::ResetHelper{}, std::forward<CaptorTupleT>(captors));
}

}  // namespace flow

#endif  // FLOW_IMPL_SYNCHRONIZER_HPP
