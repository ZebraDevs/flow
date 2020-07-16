/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file captor_state.h
 */
#ifndef FLOW_CAPTOR_STATE_OSTREAM_H
#define FLOW_CAPTOR_STATE_OSTREAM_H

// C++ Standard Library
#include <ostream>

// Flow
#include <flow/captor_state.h>

namespace flow
{

/**
 * @brief Output stream overload for <code>State</code> codes
 * @param[in,out] os  output stream
 * @param state  state code
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const State state)
{
  switch (state)
  {
  case State::RETRY:
    return os << "RETRY";
  case State::PRIMED:
    return os << "PRIMED";
  case State::ABORT:
    return os << "ABORT";
  case State::TIMEOUT:
    return os << "TIMEOUT";
  default:
    return os;
  }
}

}  // namespace flow

#endif  // FLOW_CAPTOR_STATE_OSTREAM_H
