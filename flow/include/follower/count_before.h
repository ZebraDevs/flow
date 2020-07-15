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
 * @brief Captures N-elements before the capture range lower bound, minus a delay period
 *
 *        All older elements are removed.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename AllocatorT = std::allocator<DispatchT>>
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
  template <typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT output, const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::dry_capture_policy_impl
   */
  inline State dry_capture_follower_impl(const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::abort_policy_impl
   */
  constexpr void abort_follower_impl(const stamp_type& t_abort) noexcept(true) {}

  /**
   * @copydoc Follower::reset_policy_impl
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
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template <typename DispatchT, typename LockPolicyT, typename AllocatorT>
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
