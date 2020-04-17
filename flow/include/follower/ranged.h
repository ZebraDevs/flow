/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file ranged.h
 */
#ifndef FLOW_FOLLOWER_RANGED_H
#define FLOW_FOLLOWER_RANGED_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures 1 -element before and 1 element after a sequencing range, and all element which fall between
 *
 *        Also captures all data with associated sequencing stamps between
 *        the lower and upper sequencing stamps
 *
 *        <b>Data removal:</b> Captor will remove all data before N-th element
 *        before the lower sequence stamp (<code>range.lower_stamp</code>)
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class Ranged : public Follower<Ranged<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Ranged>::stamp_type;

  /// Integer size type
  using size_type = typename CaptorTraits<Ranged>::size_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<Ranged>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param delay  the delay with which to capture
   */
  Ranged(const offset_type& delay);

  /**
   * @brief Setup constructor
   *
   * @param delay  the delay with which to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  Ranged(const offset_type& delay, const AllocatorT& alloc);

private:
  using PolicyType = Follower<Ranged<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval ABORT   If N-elements do not exist before <code>range.lower_stamp</code>
   * @retval PRIMED  If N-elements exist before <code>range.lower_stamp</code> and
   *                 M-elements exist after <code>range.upper_stamp</code>
   * @retval RETRY   If N-elements exist before <code>range.lower_stamp</code> but
   *                 M-elements do not exist after <code>range.upper_stamp</code>
   */
  template<typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT&& output, const CaptureRange<stamp_type>& range);

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
struct CaptorTraits<follower::Ranged<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/ranged.hpp>

#endif  // FLOW_FOLLOWER_RANGED_H
