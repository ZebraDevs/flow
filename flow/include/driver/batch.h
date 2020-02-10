/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file batch.h
 * @brief Defines a sliding-window driving input capture buffer
 */
#ifndef FLOW_DRIVER_BATCH_H
#define FLOW_DRIVER_BATCH_H

// C++ Standard Library
#include <memory>
#include <vector>

// Flow
#include <flow/captor.h>
#include <flow/dispatch.h>
#include <flow/driver/driver.h>

namespace flow
{
namespace driver
{

/**
 * @brief Batching input-capture object
 *
 *        Captures a sliding window of <i>N</i> elements, in sequence.
 *        Produces a target sequencing stamp range using the newest and oldest
 *        elements in the captured data window.
 * \n
*         <b>Data removal:</b> Captor will remove a single data element
 *        on each new capture attempt after N-elements have been captured
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 */
template<typename DispatchT,
         typename LockPolicyT = NoLock,
         typename AllocatorT = std::allocator<DispatchT>>
class Batch : public Driver<Batch<DispatchT, LockPolicyT, AllocatorT>>
{
public:
  /// Integer size type
  using size_type = typename CaptorTraits<Batch>::size_type;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<Batch>::stamp_type;

  /**
   * @brief Configuration constructor
   *
   * @param size  number of elements to batch before becoming ready
   *
   * @throws <code>std::invalid_argument</code> if <code>size == 0</code>
   * @throws <code>std::invalid_argument</code> if <code>CaptureOutputT</code> cannot hold \p size
   */
  explicit Batch(size_type size) noexcept(false);

  /**
   * @brief Configuration constructor
   *
   * @param size  number of elements to batch before becoming ready
   * @param alloc  dispatch object allocator with some initial state
   *
   * @throws <code>std::invalid_argument</code> if <code>size == 0</code>
   * @throws <code>std::invalid_argument</code> if <code>CaptureOutputT</code> cannot hold \p size
   */
  explicit Batch(size_type size, const AllocatorT& alloc) noexcept(false);

private:
  using PolicyType = Driver<Batch<DispatchT, LockPolicyT, AllocatorT>>;
  friend PolicyType;

  /**
   * @brief Checks if buffer is in ready state and captures data
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @retval State::PRIMED    N-elements have been captured
   * @retval State::RETRY  Captor should continue waiting for messages after prime attempt
   */
  template<typename OutputDispatchIteratorT>
  inline State capture_driver_impl(OutputDispatchIteratorT output, CaptureRange<stamp_type>& range);

  /**
   * @brief Defines behavior on <code>ABORT</code>
   * @param t_abort  sequencing stamp at which abort was signaled
   */
  inline void abort_driver_impl(const stamp_type& t_abort);

  /**
   * @brief Defines Captor reset behavior
   */
  inline void reset_driver_impl() noexcept(true) {}

  /**
   * @brief Validates captor configuration
   */
  inline void validate() const noexcept(false);

  /// Number of elements to batch
  size_type batch_size_;
};

}  // namespace driver


/**
 * @copydoc CaptorTraits
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock
 * @tparam AllocatorT  <code>DispatchT</code> allocator type
 * @tparam CaptureOutputT  output capture container type
 */
template<typename DispatchT,
         typename LockPolicyT,
         typename AllocatorT>
struct CaptorTraits<driver::Batch<DispatchT, LockPolicyT, AllocatorT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Dispatch object allocation type
  using DispatchAllocatorType = AllocatorT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;;
};

}  // namespace flow

// Flow (implementation)
#include <flow/driver/impl/batch.hpp>

#endif  // FLOW_DRIVER_BATCH_H
