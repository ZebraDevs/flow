/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file throttle.h
 */
#ifndef FLOW_DRIVER_THROTTLED_H
#define FLOW_DRIVER_THROTTLED_H

// C++ Standard Library
#include <memory>
#include <vector>

// Flow
#include <flow/captor.h>
#include <flow/dispatch.h>
#include <flow/driver/driver.h>

namespace flow
{
namespace driver
{

/**
 * @brief Throttled next element driving capture object
 *
 *        Captures the next oldest data element, limited to a max expected period. This means that some elements
 *        are skipped if the input rate indicated by data sequence stamps is higher than the throttled rate, then
 *        some messages will be ignored and not captured.
 * \n
 *        This captor produces a target sequencing stamp range using the stamp associated with
 *        the captured element (<code>range.lower_stamp == range.upper_stamp</code>)
 * \n
 *        <b>Data removal:</b> Captor will remove all elements before the captured data element
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  captured output container type
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename AllocatorT = std::allocator<DispatchT>>
class Throttled : public Driver<Throttled<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Throttled>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<Throttled>::offset_type;

  /**
   * @brief Configuration constructor
   *
   * @param throttle_period  capture throttling period
   */
  explicit Throttled(const offset_type throttle_period);

  /**
   * @brief Configuration constructor
   *
   * @param throttle_period  capture throttling period
   * @param alloc  dispatch object allocator with some initial state
   */
  explicit Throttled(const offset_type throttle_period, const AllocatorT& alloc);

private:
  using PolicyType = Driver<Throttled<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and captures data
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @retval State::PRIMED  next element has been captured
   * @retval State::RETRY  Captor should continue waiting for messages after prime attempt
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_driver_impl(OutputDispatchIteratorT output, CaptureRange<stamp_type>& range);

  /**
   * @copydoc Driver::dry_capture_policy_impl
   */
  inline State dry_capture_driver_impl(CaptureRange<stamp_type>& range) const;

  /**
   * @copydoc Driver::abort_policy_impl
   */
  inline void abort_driver_impl(const stamp_type& t_abort);

  /**
   * @copydoc Driver::reset_policy_impl
   */
  inline void reset_driver_impl();

  /// Capture throttling period
  offset_type throttle_period_;

  /// Previous captured element stamp
  stamp_type previous_stamp_;
};

}  // namespace driver


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
struct CaptorTraits<driver::Throttled<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
};

}  // namespace flow

// Flow (implementation)
#include <flow/driver/impl/throttled.hpp>

#endif  // FLOW_DRIVER_THROTTLED_H
