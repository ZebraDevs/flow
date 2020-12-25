/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file synchronizer_ostream.h
 */
#ifndef FLOW_SYNCHRONIZER_OSTREAM_H
#define FLOW_SYNCHRONIZER_OSTREAM_H

// C++ Standard Library
#include <ostream>

// Flow
#include <flow/captor_state_ostream.h>
#include <flow/dispatch_ostream.h>
#include <flow/synchronizer.h>

namespace flow
{

/**
 * @brief Output stream overload for <code>Result</code>
 * @param[in,out] os  output stream
 * @param result  Synchronizer result object
 * @return os
 */
template <typename StampT> inline std::ostream& operator<<(std::ostream& os, const Result<StampT>& result)
{
  return os << "state: " << result.state << ", range: " << result.range;
}

}  // namespace flow

#endif  // FLOW_SYNCHRONIZER_OSTREAM_H
