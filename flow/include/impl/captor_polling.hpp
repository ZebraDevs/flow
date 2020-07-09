/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 * 
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_CAPTURE_IMPL_CAPTOR_POLLING_HPP
#define FLOW_CAPTURE_IMPL_CAPTOR_POLLING_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>

namespace flow
{

/**
 * @copydoc Captor
 * @note Queue-only lock captor implementation. Implements a simple locking policy
 *       using a <code>std::lock_gaurd</code> which protects data input/output,
 *       but allows for polling with <code>Captor::capture</code>
 */
template<typename CaptorT, typename BasicLockableT>
class Captor<CaptorT, PollingLock<BasicLockableT>> : public CaptorInterface<Captor<CaptorT, PollingLock<BasicLockableT>>>
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
   *
   * @note Initializes data capacity with NO LIMITS on buffer size
   */
  Captor() : CaptorInterfaceType{0UL} {}

  /**
   * @brief Dispatch allocator constructor
   *
   * @param alloc  allocator object with some initial state
   *
   * @note Initializes data capacity with NO LIMITS on buffer size
   */
  Captor(const DispatchAllocatorType& alloc) :
    CaptorInterfaceType{0UL, alloc}
  {}

  /**
   * @brief Destructor
   * @note Releases data waits
   */
  ~Captor() = default;

private:
  /**
   * @copydoc CaptorInterface::reset
   */
  inline void reset_impl()
  {
    BasicLockableT lock{queue_mutex_};
    derived()->reset_policy_impl();
  }

  /**
   * @copydoc CaptorInterface::abort
   */
  inline void abort_impl(const stamp_type& t_abort)
  {
    BasicLockableT lock{queue_mutex_};
    derived()->abort_policy_impl(t_abort);
  }

  /**
   * @copydoc CaptorInterface::size
   */
  inline size_type size_impl() const
  {
    BasicLockableT lock{queue_mutex_};
    return CaptorInterfaceType::queue_.size();
  }

  /**
   * @copydoc CaptorInterface::inject
   */
  template<typename... DispatchConstructorArgTs>
  inline void inject_impl(DispatchConstructorArgTs&&... dispatch_args)
  {
    BasicLockableT lock{queue_mutex_};
    CaptorInterfaceType::insert_and_limit(std::forward<DispatchConstructorArgTs>(dispatch_args)...);
  }

  /**
   * @copydoc CaptorInterface::insert
   */
  template<typename FirstForwardDispatchIteratorT, typename LastForwardDispatchIteratorT>
  inline void insert_impl(FirstForwardDispatchIteratorT first, LastForwardDispatchIteratorT last)
  {
    BasicLockableT lock{queue_mutex_};
    std::for_each(first, last, [this](const DispatchType& dispatch) { CaptorInterfaceType::insert_and_limit(dispatch); });
  }

  /**
   * @copydoc CaptorInterface::set_capacity
   */
  inline void set_capacity_impl(const size_type capacity)
  {
    BasicLockableT lock{queue_mutex_};
    CaptorInterfaceType::capacity_ = capacity;
  }

  /**
   * @copydoc CaptorInterface::get_capacity
   */
  inline size_type get_capacity_impl() const
  {
    BasicLockableT lock{queue_mutex_};
    return CaptorInterfaceType::capacity_;
  }

  /**
   * @copydoc CaptorInterface::get_available_stamp_range
   */
  inline CaptureRange<stamp_type> get_available_stamp_range_impl() const
  {
    BasicLockableT lock{queue_mutex_};
    return queue_.empty() ?
           CaptureRange<stamp_type>{} :
           CaptureRange<stamp_type>{queue_.oldest_stamp(), queue_.newest_stamp()};
  }

  /**
   * @copydoc CaptorInterface::capture
   */
  template<typename OutputDispatchIteratorT, typename CaptureRangeT>
  inline State capture_impl(OutputDispatchIteratorT&& output, CaptureRangeT&& range)
  {
    BasicLockableT lock{queue_mutex_};
    return derived()->capture_policy_impl(std::forward<OutputDispatchIteratorT>(output),
                                          std::forward<CaptureRangeT>(range));
  }

  /**
   * @copydoc CaptorInterface::dry_capture
   */
  template<typename CaptureRangeT>
  inline State dry_capture_impl(CaptureRangeT&& range) const
  {
    BasicLockableT lock{queue_mutex_};
    return derived()->dry_capture_policy_impl(std::forward<CaptureRangeT>(range));
  }

  /**
   * @copydoc CaptorInterface::capture
   */
  template<typename InpectCallbackT>
  void inspect_impl(InpectCallbackT&& inspect_dispatch_cb) const
  {
    BasicLockableT lock{queue_mutex_};
    for (const auto& dispatch : CaptorInterfaceType::queue_)
    {
      inspect_dispatch_cb(dispatch);
    }
  }

  /// Mutex to protect queue ONLY
  mutable std::mutex queue_mutex_;

  using CaptorInterfaceType = CaptorInterface<Captor<CaptorT, PollingLock<BasicLockableT>>>;
  friend CaptorInterfaceType;

  FLOW_IMPLEMENT_CRTP_BASE(CaptorT);

protected:
  using CaptorInterfaceType::queue_;
};

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_POLLING_HPP
