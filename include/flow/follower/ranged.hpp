/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_FOLLOWER_RANGED_HPP
#define FLOW_FOLLOWER_RANGED_HPP

// Flow
#include <flow/follower/follower.hpp>

namespace flow
{
namespace follower
{

/**
 * @brief Captures one one element before the capture range lower bound; one element after the capture range upper
 * bound.
 *
 * All elements in between are also captured. All older elements are removed.
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
class Ranged : public Follower<Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
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
   * @param container  container object with some initial state
   * @param queue_monitor  queue monitor with some initial state
   */
  explicit Ranged(
    const offset_type& delay,
    const ContainerT& container = ContainerT{},
    const QueueMonitorT& queue_monitor = QueueMonitorT{});

private:
  using PolicyType = Follower<Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and collects data based on a target time
   *
   * @param[out] output  output data iterator
   * @param[in] range  data capture/sequencing range
   *
   * @retval ABORT   If N-elements do not exist before <code>range.lower_stamp</code>
   * @retval PRIMED  If N-elements exist before <code>range.lower_stamp</code> and
   *         M-elements exist after <code>range.upper_stamp</code>
   * @retval RETRY   If N-elements exist before <code>range.lower_stamp</code> but
   *         M-elements do not exist after <code>range.upper_stamp</code>
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

  /**
   * @brief Finds iterator after first in capture sequence
   *
   * @param range  data capture/sequencing range
   */
  inline auto find_after_first(const CaptureRange<stamp_type>& range) const;

  /**
   * @brief Finds iterator before last in capture sequence
   *
   * @param range  data capture/sequencing range
   */
  template <typename QueueIteratorT>
  inline auto find_before_last(const CaptureRange<stamp_type>& range, const QueueIteratorT after_first) const;

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
struct CaptorTraits<follower::Ranged<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
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

  /// Indicates that data from this captor will always be captured deterministically, so long as data
  /// injection is monotonically sequenced
  static constexpr bool is_capture_deterministic = true;
};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/follower/ranged.hpp>

#endif  // FLOW_FOLLOWER_RANGED_HPP
