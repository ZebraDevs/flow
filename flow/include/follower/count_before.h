/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file count_before.h
 */
#ifndef FLOW_FOLLOWER_COUNT_BEFORE_H
#define FLOW_FOLLOWER_COUNT_BEFORE_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures exactly N-data elements from a delay before the driving sequencing stamp
 *
 *        This capture buffer will capture N-elements behind the driving upper
 *        sequence stamp (<code>range.upper_stamp</code>) by some sequencing delay
 *        w.r.t a driver-provided target time. It will return all data at and before
 *        that sequencing boundary that has not previously been captured.
 * \n
 *        This capture buffer becomes ready when N-elements are available after the
 *        current driving sequencing range, minus a specified delay
 * \n
 *        Capture elements always be those closest to the delayed boundary
 * \n
 *        <b>Data removal:</b> Captor will remove all data before the oldest capture element
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class CountBefore : public Follower<CountBefore<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Integer size type
  using size_type = typename CaptorTraits<CountBefore>::size_type;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<CountBefore>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<CountBefore>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param count  number of elements before to capture
   * @param delay  the delay with which to capture
   *
   * @throws <code>std::invalid_argument</code> if <code>count == 0</code>
   */
  explicit CountBefore(const size_type count, const offset_type& delay);

  /**
   * @brief Setup constructor
   *
   * @param count  number of elements before to capture
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   *
   * @throws <code>std::invalid_argument</code> if <code>count == 0</code>
   */
  CountBefore(const size_type count, const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<CountBefore<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  if there are N Dispatch elements with a sequencing stamp greater than or
   *                 equal to the upper driving stamp, minus specified delay
   * @retval ABORT  if capture is not possible
   * @retval RETRY  otherwise
   */
  template<typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT output, const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::dry_capture_policy_impl
   */
  inline State dry_capture_follower_impl(const CaptureRange<stamp_type>& range);

  /**
   * @brief Defines behavior on <code>ABORT</code>
   * @param t_abort  sequencing stamp at which abort was signaled
   */
  constexpr void abort_follower_impl(const stamp_type& t_abort) noexcept(true) {}

  /**
   * @brief Defines Captor reset behavior
   */
  constexpr void reset_follower_impl() noexcept(true) {}

  /// Number of elements to be caputed
  size_type count_;

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
struct CaptorTraits<follower::CountBefore<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/count_before.hpp>

#endif  // FLOW_FOLLOWER_COUNT_BEFORE_H
