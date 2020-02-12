/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file count.h
 */
#ifndef FLOW_FOLLOWER_COUNT_H
#define FLOW_FOLLOWER_COUNT_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures N-elements before and M-elements after a sequencing range
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
class Count : public Follower<Count<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<Count>::stamp_type;

  /// Integer size type
  using size_type = typename CaptorTraits<Count>::size_type;

  /**
   * @brief Setup constructor
   * @param n_before  number of elements before target time to capture
   * @param m_after  number of elements before target time to capture
   */
  Count(const size_type n_before, const size_type m_after);

  /**
   * @brief Setup constructor
   * @param n_before  number of elements before target time to capture
   * @param m_after  number of elements before target time to capture
   * @param alloc  dispatch object allocator with some initial state
   */
  Count(const size_type n_before, const size_type m_after, const AllocatorT& alloc);

private:
  using PolicyType = Follower<Count<DispatchT, LockPolicyT, AllocatorT>>;
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

  /// Number of message before target to accept before ready
  size_type n_before_;

  /// Number of message after target to accept before ready
  size_type m_after_;
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
struct CaptorTraits<follower::Count<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/count.hpp>

#endif  // FLOW_FOLLOWER_COUNT_H
