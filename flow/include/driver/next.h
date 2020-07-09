/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file next.h
 */
#ifndef FLOW_DRIVER_NEXT_H
#define FLOW_DRIVER_NEXT_H

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
 * @brief Next element driving capture object
 *
 *        Captures the next oldest data element.
 *        Produces a target sequencing stamp range using the stamp associated with
 *        the captured element (<code>range.lower_stamp == range.upper_stamp</code>)
 * \n
 *        <b>Data removal:</b> Captor will remove a single data element on each new capture attempt
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  captured output container type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class Next : public Driver<Next<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Integer size type
  using size_type = typename CaptorTraits<Next>::size_type;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<Next>::stamp_type;

  /**
   * @brief Configuration constructor
   */
  explicit Next() = default;

  /**
   * @brief Configuration constructor
   * @param alloc  dispatch object allocator with some initial state
   */
  explicit Next(const AllocatorT& alloc);

private:
  using PolicyType = Driver<Next<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and captures data
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @retval State::PRIMED    next element has been captured
   * @retval State::RETRY  Captor should continue waiting for messages after prime attempt
   */
  template<typename OutputDispatchIteratorT>
  inline State capture_driver_impl(OutputDispatchIteratorT output, CaptureRange<stamp_type>& range);

  /**
   * @copydoc Driver::dry_capture_policy_impl
   */
  inline State dry_capture_driver_impl(CaptureRange<stamp_type>& range) const;

  /**
   * @brief Defines behavior on <code>ABORT</code>
   * @param t_abort  sequencing stamp at which abort was signaled
   */
  inline void abort_driver_impl(const stamp_type& t_abort);

  /**
   * @brief Defines Captor reset behavior
   */
  inline void reset_driver_impl() noexcept(true) {}
};

}  // namespace driver


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template<typename DispatchT,
         typename LockPolicyT,
         typename AllocatorT>
struct CaptorTraits<driver::Next<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
};

}  // namespace flow

// Flow (implementation)
#include <flow/driver/impl/next.hpp>

#endif  // FLOW_DRIVER_NEXT_H
