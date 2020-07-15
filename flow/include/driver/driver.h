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

#ifndef FLOW_DRIVER_DRIVER_H
#define FLOW_DRIVER_DRIVER_H

// C++ Standard Library
#include <type_traits>

// Flow
#include <flow/captor.h>
#include <flow/impl/implement_crtp_base.hpp>

namespace flow
{

// Forward declaration
template <typename PolicyT> class Driver;


/**
 * @copydoc CaptorTraits
 * @tparam PolicyT  CRTP-derived captor with specialized capture policy
 */
template <typename PolicyT> struct CaptorTraits<Driver<PolicyT>> : CaptorTraits<PolicyT>
{};


/**
 * @brief CRTP-base for Driver input-capture policies
 *
 *        Captures data produces a synchronization sequencing range used
 *        to synchronize data produced by Follower buffers
 *
 * @tparam PolicyT  CRTP-derived captor with specialized capture policy
 */
template <typename PolicyT>
class Driver : public Captor<Driver<PolicyT>, typename CaptorTraits<PolicyT>::LockPolicyType>
{
public:
  /// Data dispatch allocator type
  using DispatchAllocatorType = typename CaptorTraits<PolicyT>::DispatchAllocatorType;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<PolicyT>::stamp_type;

  /**
   * @brief Default constructor
   */
  Driver();

  /**
   * @brief Dispatch allocator constructor
   * @param alloc  dispatch object allocator with some initial state
   */
  explicit Driver(const DispatchAllocatorType& alloc);

private:
  /**
   * @brief Checks if buffer is in ready state and captures data
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @retval State::PRIMED    Data have been captured
   * @retval State::RETRY  Captor should continue waiting for messages after prime attempt
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_policy_impl(OutputDispatchIteratorT&& output, CaptureRange<stamp_type>& range);

  /**
   * @copydoc CaptorInterface::dry_capture
   * @note Sets data capture/sequencing range for following captors
   */
  inline State dry_capture_policy_impl(CaptureRange<stamp_type>& range);

  /**
   * @brief Defines Captor behavior on <code>ABORT</code>
   *
   *        Triggers data removal before \p t_abort
   *
   * @param t_abort  time at which abort was signaled
   */
  inline void abort_policy_impl(const stamp_type& t_abort);

  /**
   * @brief Defines Captor reset behavior
   */
  inline void reset_policy_impl();

  FLOW_IMPLEMENT_CRTP_BASE(PolicyT);

  using CaptorType = Captor<Driver, typename CaptorTraits<PolicyT>::LockPolicyType>;
  friend CaptorType;

protected:
  using CaptorType::queue_;
};


/**
 * @brief Checks if captor object derived from a Follower base
 * @param CaptorT  object to test
 */
template <typename CaptorT>
struct is_driver : std::integral_constant<bool, std::is_base_of<Driver<CaptorT>, CaptorT>::value>
{};

}  // namespace flow

// Flow (implementation)
#include <flow/driver/impl/driver.hpp>

#endif  // FLOW_DRIVER_DRIVER_H
