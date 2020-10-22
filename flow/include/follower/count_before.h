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
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename ContainerT = DefaultContainer<DispatchT>>
class CountBefore : public Follower<CountBefore<DispatchT, LockPolicyT, ContainerT>>
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
   * @param container  dispatch object container (non-default initialization)
   *
   * @throws <code>std::invalid_argument</code> if <code>count == 0</code>
   */
  CountBefore(const size_type count, const offset_type& delay, const ContainerT& container);

private:
  using PolicyType = Follower<CountBefore<DispatchT, LockPolicyT, ContainerT>>;
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
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam CaptureOutputT  output capture container type
 */
template <typename DispatchT, typename LockPolicyT, typename ContainerT>
struct CaptorTraits<follower::CountBefore<DispatchT, LockPolicyT, ContainerT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Underlying dispatch container type
  using DispatchContainerType = ContainerT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;

  /// Indicates that data from this captor will NOT always be captured deterministically;
  /// i.e. is always dependent on when data is injected, and when captrue is executed
  static constexpr bool is_capture_deterministic = false;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/count_before.hpp>

#endif  // FLOW_FOLLOWER_COUNT_BEFORE_H
