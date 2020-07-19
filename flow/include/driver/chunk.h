/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file chuck.h
 */
#ifndef FLOW_DRIVER_CHUNK_H
#define FLOW_DRIVER_CHUNK_H

// Flow
#include <flow/captor.h>
#include <flow/dispatch.h>
#include <flow/driver/driver.h>

namespace flow
{
namespace driver
{

/**
 * @brief Captures the next oldest data element
 *
 *        Establishes a sequencing range with where <code>range.lower_stamp</code> is the stamp of
 *        the oldest captured element, and <code>range.upper_stamp</code> is the stamp of the newest.
 *        Removes all captured elements from buffer.
 *
 * @tparam DispatchT  data dispatch type
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object or NoLock or
 * PollingLock
 * @tparam ContainerT  underlying <code>DispatchT</code> container type
 * @tparam CaptureOutputT  captured output container type
 */
template <typename DispatchT, typename LockPolicyT = NoLock, typename ContainerT = DefaultContainer<DispatchT>>
class Chunk : public Driver<Chunk<DispatchT, LockPolicyT, ContainerT>>
{
public:
  /// Integer size type
  using size_type = typename CaptorTraits<Chunk>::size_type;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<Chunk>::stamp_type;

  /**
   * @brief Configuration constructor
   *
   * @param size  number of elements to batch before becoming ready
   *
   * @throws <code>std::invalid_argument</code> if <code>size == 0</code>
   * @throws <code>std::invalid_argument</code> if <code>CaptureOutputT</code> cannot hold \p size
   */
  explicit Chunk(const size_type size) noexcept(false);

  /**
   * @brief Configuration constructor
   *
   * @param size  number of elements to batch before becoming ready
   * @param container  dispatch object container (non-default initialization)
   *
   * @throws <code>std::invalid_argument</code> if <code>size == 0</code>
   * @throws <code>std::invalid_argument</code> if <code>CaptureOutputT</code> cannot hold \p size
   */
  explicit Chunk(const size_type size, const ContainerT& container) noexcept(false);

private:
  using PolicyType = Driver<Chunk<DispatchT, LockPolicyT, ContainerT>>;
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
  template <typename OutputDispatchIteratorT>
  inline State capture_driver_impl(OutputDispatchIteratorT output, CaptureRange<stamp_type>& range);

  /**
   * @copydoc Driver::dry_capture_policy_impl
   */
  inline State dry_capture_driver_impl(CaptureRange<stamp_type>& range) const;

  /**
   * @copydoc Driver::abort_policy_impl
   */
  inline void abort_driver_impl(const stamp_type& t_abort);

  /**
   * @copydoc Driver::reset_policy_impl
   */
  inline void reset_driver_impl() noexcept(true) {}

  /**
   * @brief Validates captor configuration
   */
  inline void validate() const noexcept(false);

  /// Number of elements to batch
  size_type chunk_size_;
};

}  // namespace driver


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
struct CaptorTraits<driver::Chunk<DispatchT, LockPolicyT, ContainerT>> : CaptorTraitsFromDispatch<DispatchT>
{
  /// Underlying dispatch container type
  using DispatchContainerType = ContainerT;

  /// Thread locking policy type
  using LockPolicyType = LockPolicyT;
  ;
};

}  // namespace flow

// Flow (implementation)
#include <flow/driver/impl/chunk.hpp>

#endif  // FLOW_DRIVER_CHUNK_H
