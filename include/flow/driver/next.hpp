/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_DRIVER_NEXT_HPP
#define FLOW_DRIVER_NEXT_HPP

// Flow
#include <flow/captor.hpp>
#include <flow/dispatch.hpp>
#include <flow/driver/driver.hpp>

namespace flow
{
namespace driver
{

/**
 * @brief Captures the next oldest data element
 *
 * Establishes a sequencing range with <code>range.lower_stamp == range.upper_stamp</code> equal to
 * the captured element stamp. Removes captured element from buffer.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam QueueMonitorT  object used to monitor queue state on each insertion
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
class Next : public Driver<Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
{
public:
  /// Integer size type
  using size_type = typename CaptorTraits<Next>::size_type;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<Next>::stamp_type;

  /**
   * @brief Configuration constructor
   *
   * @param container  container object with some initial state
   */
  explicit Next(const ContainerT& container = ContainerT{}, const QueueMonitorT& queue_monitor = QueueMonitorT{});

private:
  using PolicyType = Driver<Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and captures data
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @retval State::PRIMED    next element has been captured
   * @retval State::RETRY  Captor should continue waiting for messages after prime attempt
   */
  template <typename OutputDispatchIteratorT>
  inline State capture_driver_impl(OutputDispatchIteratorT& output, CaptureRange<stamp_type>& range);

  /**
   * @copydoc Driver::locate_policy_impl
   */
  inline std::tuple<State, ExtractionRange> locate_driver_impl(CaptureRange<stamp_type>& range) const;

  /**
   * @copydoc Driver::extract_policy_impl
   */
  template <typename OutputDispatchIteratorT>
  inline void extract_driver_impl(
    OutputDispatchIteratorT& output,
    const ExtractionRange& extraction_range,
    const CaptureRange<stamp_type>& range);

  /**
   * @copydoc Driver::abort_policy_impl
   */
  inline void abort_driver_impl(const stamp_type& t_abort);

  /**
   * @copydoc Driver::reset_policy_impl
   */
  inline void reset_driver_impl() noexcept(true) {}
};

}  // namespace driver


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam QueueMonitorT  object used to monitor queue state on each insertion
 */
template <
  typename DispatchT,
  typename LockPolicyT,
  typename ContainerT,
  typename QueueMonitorT,
  typename AccessStampT,
  typename AccessValueT>
struct CaptorTraits<driver::Next<DispatchT, LockPolicyT, ContainerT, QueueMonitorT, AccessStampT, AccessValueT>>
    : CaptorTraitsFromDispatch<DispatchT>
{
  /// Underlying dispatch container type
  using DispatchContainerType = ContainerT;

  /// Queue monitor type
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
#include <flow/impl/driver/next.hpp>

#endif  // FLOW_DRIVER_NEXT_HPP
