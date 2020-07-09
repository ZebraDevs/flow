/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file matched_stamp.h
 */
#ifndef FLOW_FOLLOWER_MATCHED_STAMP_H
#define FLOW_FOLLOWER_MATCHED_STAMP_H

// Flow
#include <flow/follower/follower.h>

namespace flow
{
namespace follower
{

/**
 * @brief Captures next inputs with a sequencing stamp which exactly matches the
 *        upper stamp of driving range
 *
 *        If the oldest data in the queue is newer than the lower driving time stamp
 *        (<code>range.lower_stamp</code>), the capture sequence is aborted. If all data is older than the
 *        upper driving time stamp (<code>range.upper_stamp</code>) this captor will continue to wait
 *        for data with an exact stamp
 * \n
 *        with the same stamp, then <i>ALL</i> such elements will be captured
 *        If the driving time stamp is a range, all data elements with stamps which fall into
 *        that range will be captured
 * \n
 *        <b>Data removal:</b> Captor will remove all data at and before the captured element
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or PollingLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  captured output container type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class MatchedStamp : public Follower<MatchedStamp<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<MatchedStamp>::stamp_type;

  /**
   * @brief Default constructor
   */
  MatchedStamp() = default;

  /**
   * @brief Setup constructor
   * @param alloc  dispatch object allocator with some initial state
   */
  explicit MatchedStamp(const AllocatorT& alloc);

private:
  using PolicyType = Follower<MatchedStamp<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  If element(s) with sequencing stamp equal to
   *                 <code>range.upper_stamp</code> is available
   * @retval RETRY   If only element(s) with sequencing stamp less than
   *                 <code>range.upper_stamp</code> is available or queue is empty
   * @retval ABORT   If only element(s) with sequencing stamp greater than
   *                 <code>range.upper_stamp</code> is available
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
  inline void abort_follower_impl(const stamp_type& t_abort);

  /**
   * @brief Defines Captor reset behavior
   */
  inline void reset_follower_impl() noexcept(true) {}
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
struct CaptorTraits<follower::MatchedStamp<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
};

}  // namespace flow

// Flow (implementation)
#include <flow/follower/impl/matched_stamp.hpp>

#endif  // FLOW_FOLLOWER_MATCHED_STAMP_H
