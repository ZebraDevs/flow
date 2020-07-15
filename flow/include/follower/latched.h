/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file latched.h
 */
#ifndef FLOW_FOLLOWER_LATCHED_H
#define FLOW_FOLLOWER_LATCHED_H

// Flow
#include <flow/follower/follower.h>
#include <flow/impl/optional.hpp>

namespace flow
{
namespace follower
{

/**
 * @brief Captures one element before the capture range lower bound, minus a minimum period
 *
 *        All older elements are removed. If no newer elements are present on the next capture attempt,
 *        then the last captured element is returned. If a newer element is present on a subsequent capture attempt,
 *        meeting the aforementioned qualifications, this elements is captured and replaces "latched" element state.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 *
 * @note Latched won't behave non-deterministically if actual input period (difference between successive
 *       dispatch stamps) is greater than <code>min_period</code>. However, newer data values will not be captured if
 * they are within a <code>min_period<code> offset of <code>range.lower_stamp</code>
 *
 * @warn Latched may never enter a READY state if data never becomes available. Calling application may need to
 * implement a synchronization timeout behavior
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename AllocatorT = std::allocator<DispatchT>>
class Latched : public Follower<Latched<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Latched>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<Latched>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param min_period  minimum expected difference between data stamps
   */
  Latched(const offset_type min_period);

  /**
   * @brief Setup constructor
   *
   * @param min_period  minimum expected difference between data stamps
   * @param alloc  dispatch object allocator with some initial state
   *
   * @throw <code>std::invalid_argument</code> if <code>m_after < 1</code>
   */
  Latched(const offset_type min_period, const AllocatorT& alloc);

private:
  using PolicyType = Follower<Latched<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  If data element is available at or before <code>range.lower_stamp - min_period</code>
   * @retval RETRY   If data element is not available at or before <code>range.lower_stamp - min_period</code>
   * @retval ABORT   If data elements are only available after <code>range.lower_stamp - min_period</code>
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT output, const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::dry_capture_policy_impl
   */
  inline State dry_capture_follower_impl(const CaptureRange<stamp_type>& range);

  /**
   * @brief Defines behavior on <code>ABORT</code>
   *
   * @param t_abort  sequencing stamp at which abort was signaled
   */
  inline void abort_follower_impl(const stamp_type& t_abort);

  /**
   * @copydoc Follower::reset_policy_impl
   *
   * @note clears latched data
   */
  inline void reset_follower_impl() noexcept(true);

  /// Latched dispatch
  ::flow::optional<DispatchT> latched_;

  /// Number of message before target to accept before ready
  offset_type min_period_;
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
struct CaptorTraits<follower::Latched<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/latched.hpp>

#endif  // FLOW_FOLLOWER_LATCHED_H
