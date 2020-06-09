/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file captor.h
 */
#ifndef FLOW_CAPTOR_H
#define FLOW_CAPTOR_H

// C++ Standard Library
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>

// Flow
#include <flow/captor_state.h>
#include <flow/dispatch.h>
#include <flow/dispatch_queue.h>
#include <flow/impl/implement_crtp_base.hpp>

namespace flow
{

/**
 * @brief Underlying captor implementation
 *
 *        Implements basic captor qualities and queuing interfaces
 */
template<typename Derived>
class CaptorBase;


/**
 * @brief Stand-in type used to signify that captors will be used in a single-threaded context
 *
 *        Used in place of a TimedLockable (https://en.cppreference.com/w/cpp/named_req/TimedLockable)
 *        object used to specify a locking policy
 */
struct NoLock {};


/**
 * @brief Stand-in type used to signify that captors will be used in a threaded context, but will not wait for data
 *
 *        Captors use <code>BasicLockableT</code> to protect data input/output from the capture queue, but
 *        do not wait on a condition variable for new inputs before attempting to run a synchronization policy.
 *        This allows for polling with the <code>Captor::capture</code> method
 * \n
 *        See https://en.cppreference.com/w/cpp/named_req/TimedLockable for more information on <code>BasicLockableT</code>
 *        criteria
 */
template<typename BasicLockableT = std::lock_guard<std::mutex>>
struct PollingLock
{
  using type = BasicLockableT;
};


/**
 * @brief Basic captor traits struct with common type information from data dispatch object
 *
 * @tparam DispatchT  data dispatch object type
 */
template<typename DispatchT>
struct CaptorTraitsFromDispatch
{
  /// Dispatch type
  using DispatchType = DispatchT;

  /// Dispatch data value type
  using value_type = typename DispatchTraits<DispatchT>::value_type;

  /// Dispatch stamp type
  using stamp_type = typename DispatchTraits<DispatchT>::stamp_type;

  /// Duration/offset type compatible with <code>stamp_type</code>
  using offset_type = typename DispatchTraits<DispatchT>::offset_type;

  /// Integer size type
  using size_type = std::size_t;
};


/**
 * @brief Traits struct for captor types
 *
 *        Requires:
 *        - <code>DispatchType</code> : data dispatch object type
 *        - <code>DispatchAllocatorType</code> : allocator for <code>DispatchType</code>
 *        - <code>value_type</code> : data value type
 *        - <code>stamp_type</code> : sequence stamp type
 *        - <code>size_type</code> : integer sizing type
 *
 * @tparam CaptorT  captor type with CRTP base <code>Captor</code>
 */
template<typename CaptorT>
struct CaptorTraits;


/**
 * @brief CRTP-base which defines basic captor interface
 */
template<typename CaptorT>
class CaptorInterface
{
public:
  /// Data dispatch type
  using DispatchType = typename CaptorTraits<CaptorT>::DispatchType;

  /// Data dispatch allocator type
  using DispatchAllocatorType = typename CaptorTraits<CaptorT>::DispatchAllocatorType;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<CaptorT>::stamp_type;

  /// Integer size type
  using size_type = typename CaptorTraits<CaptorT>::size_type;

  /**
   * @brief Capacity setup constructor
   *
   * @param capacity  maximum buffer capacity
   */
  explicit CaptorInterface(const size_type capacity);

  /**
   * @brief Full setup constructor
   *
   * @param capacity  maximum buffer capacity
   * @param alloc  dispatch allocator type for underlying queue
   */
  explicit CaptorInterface(const size_type capacity, const DispatchAllocatorType& alloc);

  /**
   * @brief Clears all captor data and resets all states
   */
  inline void reset()
  {
    derived()->reset_impl();
  }

  /**
   * @brief Returns the number of buffered elements
   */
  inline size_type size() const
  {
    return derived()->size_impl();
  }

  /**
   * @brief Injects new data into Captor queue
   *
   *        Data is automatically removed from <code>queue_</code> when its
   *        size is in excess of the size specified by <code>capacity_</code>
   *
   * @tparam DispatchConstructorArgTs...  dispatch constructor argument types
   *
   * @param dispatch_args  dispatch constructor arguments
   */
  template<typename... DispatchConstructorArgTs>
  inline void inject(DispatchConstructorArgTs&&... dispatch_args)
  {
    return derived()->inject_impl(std::forward<DispatchConstructorArgTs>(dispatch_args)...);
  }

  /**
   * @brief Injects a range of new data into Captor queue
   *
   *        Data is automatically removed from <code>queue_</code> when its
   *        size is in excess of the size specified by <code>capacity_</code>
   *
   * @tparam FirstForwardDispatchIteratorT  forward iterator type for <code>DispatchType</code> elements
   * @tparam LastForwardDispatchIteratorT  forward iterator type for <code>DispatchType</code> elements
   *
   * @param dispatch_args  dispatch constructor arguments
   */
  template<typename FirstForwardDispatchIteratorT, typename LastForwardDispatchIteratorT>
  inline void insert(FirstForwardDispatchIteratorT&& first, LastForwardDispatchIteratorT&& last)
  {
    return derived()->insert_impl(std::forward<FirstForwardDispatchIteratorT>(first),
                                  std::forward<LastForwardDispatchIteratorT>(last));
  }

  /**
   * @brief Defines Captor behavior on <code>ABORT</code>
   *
   *        Triggers data removal before \p t_abort
   *
   * @param t_abort  time at which abort was signaled
   */
  inline void abort(const stamp_type& t_abort)
  {
    return derived()->abort_impl(t_abort);
  }

  /**
   * @brief Sets the maximum number of elements
   *
   *        This is the number of data elements which can be buffered
   *        in the capture queue before being discarded
   *
   * @param capacity  maximum number of buffered elements; <code>capacity == 0</code>
   *                  signifies that there will be no limit on buffer capacity
   */
  inline void set_capacity(const size_type capacity)
  {
    return derived()->set_capacity_impl(capacity);
  }

  /**
   * @brief Gets the maximum number of elements
   *
   *        This is the number of data elements which can be buffered
   *        in the capture queue before being discarded
   *
   * @see <code>set_capacity</code>
   */
  inline size_type get_capacity() const
  {
    return derived()->get_capacity_impl();
  }

  /**
   * @brief Gets the time range between oldest/newest buffered messages
   */
  inline CaptureRange<stamp_type> get_available_stamp_range() const
  {
    return derived()->get_available_stamp_range_impl();
  }

  /**
   * @brief Waits for ready state and captures inputs
   *
   * @tparam OutputDispatchIteratorT  output iterator type for a value type which supports assignment with <code>DispatchType</code>
   * @tparam CaptureRangeT  message capture stamp range type
   * @tparam ClockT  clock type associated with time-point representation
   * @tparam DurationT  duration type associated with time-point representation
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   * @param timeout  time to stop waiting for data
   *
   * @return capture directive code
   */
  template<typename OutputDispatchIteratorT, typename CaptureRangeT, typename ClockT, typename DurationT>
  inline State capture(OutputDispatchIteratorT&& output,
                       CaptureRangeT&& range,
                       const std::chrono::time_point<ClockT, DurationT> timeout = std::chrono::time_point<ClockT, DurationT>::max())
  {
    return derived()->capture_impl(std::forward<OutputDispatchIteratorT>(output),
                                   std::forward<CaptureRangeT>(range),
                                   timeout);
  }

  /**
   * @brief Waits for ready state and captures inputs
   *
   * @tparam OutputDispatchIteratorT  output iterator type for a value type which supports assignment with <code>DispatchType</code>
   * @tparam CaptureRangeT  message capture stamp range type
   *
   * @param[out] output  output data iterator
   * @param[in,out] range  data capture/sequencing range
   *
   * @return capture directive code
   */
  template<typename OutputDispatchIteratorT, typename CaptureRangeT>
  inline State capture(OutputDispatchIteratorT&& output,
                       CaptureRangeT&& range)
  {
    return derived()->capture_impl(std::forward<OutputDispatchIteratorT>(output),
                                   std::forward<CaptureRangeT>(range));
  }

  /**
   * @brief Runs inspection callback all messages available in the current queue
   *
   *        The queue and its contents will be immutable during inspection
   *
   * @tparam InpectCallbackT  queue inspection callback type which can be called as
   *                          <code>cb(const DispatchType& dispatch)</code>
   *
   * @param inspect_dispatch_cb  callback invoked for each available dispatch
   */
  template<typename InpectCallbackT>
  inline void inspect(InpectCallbackT&& inspect_dispatch_cb) const
  {
    return derived()->inspect_impl(std::forward<InpectCallbackT>(inspect_dispatch_cb));
  }

  // Sanity check to ensure that DispatchType is copyable
  static_assert(std::is_copy_constructible<DispatchType>(), "'DispatchType' must be a copyable type");

protected:
  /**
   * @brief Inserts data into queue and limit queue size to capacity, if applicable
   *
   * @param args  args forward to <code>DispatchQueue::insert</code>
   *
   * @return capture directive code
   */
  template<typename... InsertArgTs>
  inline void insert_and_limit(InsertArgTs&&... args);

  /// Data dispatch queue
  DispatchQueue<DispatchType, DispatchAllocatorType> queue_;

  /// Buffered data capacity
  size_type capacity_;

  FLOW_IMPLEMENT_CRTP_BASE(CaptorT);
};


/**
 * @brief CRTP-base for input capture buffers with a specific data lock policy
 *
 * @tparam CaptorT  CRTP-derived Captor type
 * @tparam LockableT  a TimedLockable (https://en.cppreference.com/w/cpp/named_req/TimedLockable) object;
                      specializations are available which replace <code>LockableT</code> with <code>NoLock</code> or <code>PollingLock</code>
 */
template<typename CaptorT, typename LockableT>
class Captor : public CaptorInterface<Captor<CaptorT, LockableT>>
{
public:
  /// Data dispatch type
  using DispatchType = typename CaptorTraits<CaptorT>::DispatchType;

  /// Data dispatch allocator type
  using DispatchAllocatorType = typename CaptorTraits<CaptorT>::DispatchAllocatorType;

  /// Data stamp type
  using stamp_type = typename CaptorTraits<CaptorT>::stamp_type;

  /// Integer size type
  using size_type = typename CaptorTraits<CaptorT>::size_type;

  /**
   * @brief Default constructor
   */
  Captor();

  /**
   * @brief Dispatch allocator constructor
   *
   * @param alloc  allocator object with some initial state
   */
  explicit Captor(const DispatchAllocatorType& alloc);

  /**
   * @brief Destructor
   * @note Releases data waits
   */
  ~Captor();

private:
  /**
   * @copydoc CaptorInterface::reset
   */
  inline void reset_impl();

  /**
   * @copydoc CaptorInterface::size
   */
  inline size_type size_impl() const;

  /**
   * @copydoc CaptorInterface::inject
   */
  template<typename... DispatchConstructorArgTs>
  inline void inject_impl(DispatchConstructorArgTs&&... dispatch_args);

  /**
   * @copydoc CaptorInterface::insert
   */
  template<typename FirstForwardDispatchIteratorT, typename LastForwardDispatchIteratorT>
  inline void insert_impl(FirstForwardDispatchIteratorT first, LastForwardDispatchIteratorT last);

  /**
   * @copydoc CaptorInterface::abort
   */
  inline void abort_impl(const stamp_type& t_abort);

  /**
   * @copydoc CaptorInterface::capture
   */
  template<typename OutputDispatchIteratorT, typename CaptureRangeT, typename ClockT, typename DurationT>
  inline State capture_impl(OutputDispatchIteratorT&& output,
                            CaptureRangeT&& range,
                            const std::chrono::time_point<ClockT, DurationT> timeout);

  /**
   * @copydoc CaptorInterface::inspect
   */
  template<typename InpectCallbackT>
  inline void inspect_impl(InpectCallbackT&& inspect_dispatch_cb) const;

  /**
   * @copydoc CaptorInterface::set_capacity
   */
  inline void set_capacity_impl(const size_type capacity);

  /**
   * @copydoc CaptorInterface::get_capacity
   */
  inline size_type get_capacity_impl() const;

  /**
   * @copydoc CaptorInterface::get_available_stamp_range
   */
  inline CaptureRange<stamp_type> get_available_stamp_range_impl() const;

  /// Mutex to protect queue and captures
  mutable std::mutex capture_mutex_;

  /// Flag used to indicate that capture loop should continue
  volatile bool capturing_ = true;

  /**
   * @brief Condition variable used to wait for data
   *
   *        Uses <code>std::condition_variable</code> when <code>LockableT == std::unqiue_lock</code>
   *        as is may be more performant than <code>std::condition_variable_any</code> according to
   *        https://en.cppreference.com/w/cpp/thread/condition_variable_any
   */
  mutable typename std::conditional<
    std::is_same<std::unique_lock<std::mutex>, LockableT>::value,
    std::condition_variable,
    std::condition_variable_any
  >::type capture_cv_;

  using CaptorInterfaceType = CaptorInterface<Captor<CaptorT, LockableT>>;
  friend CaptorInterfaceType;

  FLOW_IMPLEMENT_CRTP_BASE(CaptorT);

protected:
  using CaptorInterfaceType::queue_;
};


/**
 * @copydoc CaptorTraits
 *
 * @tparam PolicyT  CRTP-derived captor with specialized capture policy
 */
template<typename CaptorT, typename LockableT>
struct CaptorTraits<Captor<CaptorT, LockableT>> : CaptorTraits<CaptorT> {};


/**
 * @brief Checks if captor object derived from a Driver base
 *
 * @tparam CaptorT  object to test
 */
template<typename CaptorT>
struct is_driver;


/**
 * @brief Checks if captor object derived from a Follower base
 *
 * @tparam CaptorT  object to test
 */
template<typename CaptorT>
struct is_follower;


/**
 * @brief Checks if <code>LockableT</code> is of type NoLock
 *
 * @tparam CaptorT  object to test
 */
template<typename LockableT>
struct is_no_lock : std::integral_constant<bool, std::is_same<LockableT, NoLock>::value> {};


/**
 * @brief Checks if <code>LockableT</code> is instance of PollingLock
 *
 * @tparam CaptorT  object to test
 */
template<typename LockableT>
struct is_polling_lock : std::integral_constant<bool, false> {};


/**
 * @copydoc is_polling_lock
 */
template<typename LockableT>
struct is_polling_lock<PollingLock<LockableT>> : std::integral_constant<bool, true> {};


/**
 * @brief Checks if captor is serviced by polling capture
 *
 * @tparam CaptorT  object to test
 */
template<typename CaptorT>
struct is_polling : std::integral_constant<
  bool,
  is_polling_lock<typename CaptorTraits<CaptorT>::LockPolicyType>::value or
  is_no_lock<typename CaptorTraits<CaptorT>::LockPolicyType>::value>
{};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/captor_interface.hpp>
#include <flow/impl/captor_lockable.hpp>
#include <flow/impl/captor_nolock.hpp>
#include <flow/impl/captor_polling.hpp>

#endif  // FLOW_CAPTOR_H
