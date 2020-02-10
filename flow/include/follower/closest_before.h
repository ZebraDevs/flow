/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl, Derek King
 *
 * @file closest_before.h
 */
#ifndef FLOW_FOLLOWER_CLOSEST_BEFORE_H
#define FLOW_FOLLOWER_CLOSEST_BEFORE_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures next closest data element based on a known input rate
 *
 *        This capture buffer will search for data whose sequencing stamp is closest
 *        to the driving stamp (<code>range.upper_stamp</code>) which also occured before the
 *        upper driving stamp.
 * \n
 *        If elements are available, but it is likely that incoming element will
 *        be closer to the target stamp (based on an expected sequencing rate provided
 *        on setup) the then buffer will wait for this input before becoming ready.
 * \n
 *        If an expected input does not arrive, and an element arrives after the upper driving
 *        sequence stamp, the last nearest data available within the specified time window
 *        will be used. If no such data is available
 * \n
 *        <b>Data removal:</b> Captor will remove all data before the resolved
 *        closest data element
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class ClosestBefore : public Follower<ClosestBefore<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<ClosestBefore>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<ClosestBefore>::offset_type;

  /**
   * @brief Setup constructor
   * @param period  expected period between successive data element
   * @param delay  the delay with which to capture
   */
  ClosestBefore(const offset_type& period, const offset_type& delay);

  /**
   * @brief Setup constructor
   * @param period  expected half-period between successive data elements
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  ClosestBefore(const offset_type& period, const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<ClosestBefore<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval ABORT   If next closest element hase sequencing stamp greater than <code>range.upper_stamp</code>
   * @retval PRIMED  If next closest element to is available
   * @retval RETRY   Element with sequence stamp greater than <code>range.upper_stamp</code> exists, and
   *                 there is a data element within the expected duration window before
   *                 <code>range.upper_stamp</code>
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

  /// Expected update period
  offset_type period_;

  /// Capture delay
  offset_type delay_;
};

}  // namespace follower


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template<typename DispatchT,
         typename LockPolicyT,
         typename AllocatorT>
struct CaptorTraits<follower::ClosestBefore<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/closest_before.hpp>

#endif  // FLOW_FOLLOWER_CLOSEST_BEFORE_H
