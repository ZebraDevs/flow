/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_FOLLOWER_COUNT_BEFORE_HPP
#define FLOW_FOLLOWER_COUNT_BEFORE_HPP

// Flow
#include <flow/follower/follower.hpp>

namespace flow
{
namespace follower
{

/**
 * @brief Captures N-elements before the capture range lower bound, minus a delay period
 *
 * All older elements are removed.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam QueueMonitorT  object used to monitor queue state on each insertion; used to precondition capture
 * @tparam AccessStampT  custom stamp access
 * @tparam AccessValueT  custom value access
 */
template <
  typename DispatchT,
  typename LockPolicyT = NoLock,
  typename ContainerT = DefaultContainer<DispatchT>,
  typename QueueMonitorT = DefaultDispatchQueueMonitor,
  typename AccessStampT = DefaultStampAccess,
  typename AccessValueT = DefaultValueAccess>
class CountBefore
    : public Follower<CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
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
   * @param container  container object with some initial state
   * @param queue_monitor  queue monitor with some initial state
   *
   * @throws <code>std::invalid_argument</code> if <code>count == 0</code>
   */
  CountBefore(
    const size_type count,
    const offset_type& delay,
    const ContainerT& container = ContainerT{},
    const QueueMonitorT& queue_monitor = QueueMonitorT{});

private:
  using PolicyType =
    Follower<CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval PRIMED  if there are N Dispatch elements with a sequencing stamp greater than or
   *         equal to the upper driving stamp, minus specified delay
   * @retval ABORT  if capture is not possible
   * @retval RETRY  otherwise
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
 * @tparam QueueMonitorT queue monitor/capture preconditioning type
 */
template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
struct CaptorTraits<
  follower::CountBefore<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
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
#include <flow/impl/follower/count_before.hpp>

#endif  // FLOW_FOLLOWER_COUNT_BEFORE_HPP
