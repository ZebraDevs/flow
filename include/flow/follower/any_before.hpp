/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Levon Avagyan, Brian Cairl
 */
#ifndef FLOW_FOLLOWER_ANY_BEFORE_HPP
#define FLOW_FOLLOWER_ANY_BEFORE_HPP

// Flow
#include <flow/follower/follower.hpp>

namespace flow
{
namespace follower
{

/**
 * @brief Captures all data elements from a delay before the driving sequencing stamp
 *
 * This capture buffer will capture data which is behind the driving upper
 * sequence stamp (<code>range.upper_stamp</code>) by some sequencing delay
 * w.r.t a driver-provided target time. It can be configured to return all data at and before,
 * or strictly just before, that sequencing boundary that has not previously been captured.
 * \n
 * This capture buffer is always ready, and will always return with a PRIMED state,
 * regardless of whether or not there is data available to capture.
 * \n
 * <b>Data removal:</b> Captor will remove all data before the driving time
 * message minus the delay
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam QueueMonitorT  object used to monitor queue state on each insertion; used to precondition capture
 * @tparam AccessStampT  custom stamp access
 * @tparam AccessValueT  custom value access
 *
 * @warn This captor WILL NOT behave deterministically if all data is not available before capture time minus
 *       the specified delay. As such, setting the delay properly will alleviate non-deterministic behavior.
 *       This is the only <i>optional</i> captor, and should be used with great caution.
 */
template <
  typename DispatchT,
  typename LockPolicyT = NoLock,
  typename ContainerT = DefaultContainer<DispatchT>,
  typename QueueMonitorT = DefaultDispatchQueueMonitor,
  typename AccessStampT = DefaultStampAccess,
  typename AccessValueT = DefaultValueAccess>
class AnyBefore
    : public Follower<AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
{
public:
  /// Data stamp type
  using stamp_type = typename CaptorTraits<AnyBefore>::stamp_type;

  /// Data stamp duration type
  using offset_type = typename CaptorTraits<AnyBefore>::offset_type;

  /**
   * @brief Setup constructor
   *
   * @param delay  the delay with which to capture
   * @param inclusive_capture_boundary  enable message capture exactly at sequencing boundary
   * @param container  container object with some initial state
   * @param queue_monitor  queue monitor with some initial state
   */
  explicit AnyBefore(
    const offset_type& delay,
    const bool inclusive_capture_boundary = false,
    const ContainerT& container = ContainerT{},
    const QueueMonitorT& queue_monitor = QueueMonitorT{});

private:
  using PolicyType = Follower<AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  always
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_follower_impl(OutputDispatchIteratorT& output, const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::locate_policy_impl
   */
  inline std::tuple<State, ExtractionRange> locate_follower_impl(const CaptureRange<stamp_type>& range) const;

  /**
   * @copydoc Follower::extract_policy_impl
   */
  template <typename OutputDispatchIteratorT>
  inline void extract_follower_impl(
    OutputDispatchIteratorT& output,
    const ExtractionRange& extraction_range,
    const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Follower::abort_policy_impl
   */
  inline void abort_follower_impl(const stamp_type& t_abort);

  /**
   * @copydoc Follower::reset_policy_impl
   */
  inline void reset_follower_impl() noexcept(true) {}

  /// Capture delay
  offset_type delay_;

  /// Include message capture at the sequencing boundary
  const bool inclusive_capture_boundary_;
};

}  // namespace follower


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam QueueMonitorT queue monitor/capture preconditioning type
 */
template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
struct CaptorTraits<follower::AnyBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
    : CaptorTraitsFromDispatch<DispatchT>
{
  /// Underlying dispatch container type
  using DispatchContainerType = ContainerT;

  /// Queue monitor/capture preconditioning type
  using DispatchQueueMonitorType = QueueMonitorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;

  /// Stamp access implementation
  using AccessStampType = AccessStampT;

  /// Value access implementation
  using AccessValueType = AccessValueT;

  /// Indicates that data from this captor will NOT always be captured deterministically;
  /// i.e. is always dependent on when data is injected, and when captrue is executed
  static constexpr bool is_capture_deterministic = false;
};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/follower/any_before.hpp>

#endif  // FLOW_FOLLOWER_ANY_BEFORE_HPP
