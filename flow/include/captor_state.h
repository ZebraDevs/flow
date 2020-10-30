/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file captor_state.h
 */
#ifndef FLOW_CAPTOR_STATE_H
#define FLOW_CAPTOR_STATE_H

namespace flow
{

/**
 * @brief Evaluated Captor state
 *
 *        These are used by captors to direct internal data capture behavior
 */
enum class State : int
{
  RETRY,  ///< Captor should continue waiting for data after prime attempt
  PRIMED,  ///< Captor has captured data and its ready
  ABORT,  ///< Captor has requested to abort current capture attempt
  TIMEOUT,  ///< Captor has hit a data-wait timeout
  ERROR_DRIVER_LOWER_BOUND_EXCEEDED,  ///< Error code used to indicate that driving violated external lower bound
  _N_STATES,  ///< Total number of captor states
};

}  // namespace flow

#endif  // FLOW_CAPTOR_STATE_H
