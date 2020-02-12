/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file followers.h
 * @brief Includes all follower captor implementations
 */
#ifndef FLOW_FOLLOWERS_H
#define FLOW_FOLLOWERS_H

// Flow
#include <flow/follower/before.h>
#include <flow/follower/count.h>
#include <flow/follower/closest_before.h>
#include <flow/follower/exact.h>
#include <flow/follower/latched.h>

namespace flow
{
/**
 * @brief Synchronization buffers with sequencing range-dependent policies
 */
namespace follower {}

}  // flow

#endif  // FLOW_FOLLOWERS_H
