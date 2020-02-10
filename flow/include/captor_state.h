/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file captor_state.h
 */
#ifndef FLOW_CAPTOR_STATE_HPP
#define FLOW_CAPTOR_STATE_HPP

// C++ Standard Library
#include <array>
#include <cstdint>
#include <ostream>
#include <iomanip>

namespace flow
{

/**
 * @brief Evaluated Captor state
 *
 *        These are used by captors to direct internal data capture behavior
 */
enum class State : int
{
  RETRY,         ///< Captor should continue waiting for data after prime attempt
  PRIMED,        ///< Captor has captured data and its ready
  ABORT,         ///< Captor has requested to abort current capture attempt
  TIMEOUT,       ///< Captor has hit a data-wait timeout
  _N_STATES,     ///< Total number of captor states
};


/**
 * @brief Output stream overload for <code>State</code> codes
 * @param[in,out] os  output stream
 * @param state  state code
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, State state)
{
  switch (state)
  {
    case State::RETRY    : return os << "RETRY";
    case State::PRIMED   : return os << "PRIMED";
    case State::ABORT    : return os << "ABORT";
    case State::TIMEOUT  : return os << "TIMEOUT";
    default: return os;
  }
}

}  // namespace flow

#endif  // FLOW_CAPTOR_STATE_HPP
