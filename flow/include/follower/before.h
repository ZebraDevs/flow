/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 *
 * @file before.h
 */
#ifndef FLOW_FOLLOWER_BEFORE_H
#define FLOW_FOLLOWER_BEFORE_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures all data elements from a delay before the driving sequencing stamp
 *
 *        This capture buffer will capture data which is behind the driving upper
 *        sequence stamp (<code>range.upper_stamp</code>) by some sequencing delay
 *        w.r.t a driver-provided target time. It will return all data at and before
 *        that sequencing boundary that has not previously been captured.
 * \n
 *        This capture buffer becomes ready when any data with a sequencing stamp greater
 *        than or equal to the upper sequencing stamp, minus the specified delay duration,
 *        is available.
 * \n
 *        <b>Data removal:</b> Captor will remove all data before the driving time
 *        message minus the delay
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class Before : public Follower<Before<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Before>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<Before>::offset_type;

  /**
   * @brief Setup constructor
   * @param delay  the delay with which to capture
   */
  explicit Before(const offset_type& delay);

  /**
   * @brief Setup constructor
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  Before(const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<Before<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  if there is a Dispatch element with a sequencing stamp greater than or
   *                 equal to the upper driving stamp, minus specified delay
   * @retval RETRY  otherwise
   */
  template<typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT output, const CaptureRange<stamp_type>& range);

  /**
   * @brief Defines behavior on <code>ABORT</code>
   * @param t_abort  sequencing stamp at which abort was signaled
   */
  inline void abort_follower_impl(const stamp_type& t_abort);

  /**
   * @brief Defines Captor reset behavior
   */
  inline void reset_follower_impl() noexcept(true) {}

  /// Capture delay
  offset_type delay_;
};

}  // namespace follower


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
struct CaptorTraits<follower::Before<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/before.hpp>

#endif  // FLOW_FOLLOWER_BEFORE_H
