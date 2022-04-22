/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_CAPTOR_STATE_HPP
#define FLOW_CAPTOR_STATE_HPP

// C++ Standard Library
#include <tuple>

namespace flow
{

/**
 * @brief Evaluated Captor state
 *
 * These are used by captors to direct internal data capture behavior
 */
enum class State : int
{
  // basic execution state codes
  RETRY,  ///< Captor should continue waiting for data after prime attempt
  PRIMED,  ///< Captor has captured data and its ready
  ABORT,  ///< Captor has requested to abort current capture attempt
  TIMEOUT,  ///< Captor has hit a data-wait timeout

  // special execution state codes
  ERROR_DRIVER_LOWER_BOUND_EXCEEDED,  ///< Error code used to indicate that driving violated external lower bound
  SKIP_FRAME_QUEUE_PRECONDITION,  ///< Code use to indicate that the current sync frame is to be skipped, namely when
                                  ///< when a queue monitor object is used to precondition capture
  _N_STATES,  ///< Total number of captor states
};

template <typename... OtherTs> constexpr bool operator==(const State lhs, const std::tuple<State, OtherTs...>& rhs)
{
  return lhs == std::get<0>(rhs);
}

template <typename... OtherTs> constexpr bool operator!=(const State lhs, const std::tuple<State, OtherTs...>& rhs)
{
  return lhs != std::get<0>(rhs);
}

}  // namespace flow

#endif  // FLOW_CAPTOR_STATE_HPP
