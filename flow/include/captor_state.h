// Copyright (C) 2020, Fetch Robotics Inc.
//
// This file is part of Flow.
//
// Flow is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Flow is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Flow.  If not, see <https://www.gnu.org/licenses/>.

#ifndef FLOW_CAPTOR_STATE_HPP
#define FLOW_CAPTOR_STATE_HPP

// C++ Standard Library
#include <array>
#include <cstdint>
#include <iomanip>
#include <ostream>

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
  _N_STATES,  ///< Total number of captor states
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

#endif  // FLOW_CAPTOR_STATE_HPP
