/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 * 
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_CAPTURE_IMPL_CAPTOR_NOLOCK_HPP
#define FLOW_CAPTURE_IMPL_CAPTOR_NOLOCK_HPP

// C++ Standard Library
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>

namespace flow
{

/**
 * @copydoc Captor
 * @note No-lock captor implementation
 */
template<typename CaptorT>
class Captor<CaptorT, NoLock> : public CaptorInterface<Captor<CaptorT, NoLock>>
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
    derived()->reset_policy_impl();
  }

  /**
   * @copydoc CaptorInterface::abort
   */
  inline void abort_impl(const stamp_type& t_abort)
  {
    derived()->abort_policy_impl(t_abort);
  }

  /**
   * @copydoc CaptorInterface::size
   */
  inline size_type size_impl() const
  {
    return CaptorInterfaceType::queue_.size();
  }

  /**
   * @copydoc CaptorInterface::inject
   */
  inline void inject_impl(const DispatchType& dispatch)
  {
    CaptorInterfaceType::insert_and_limit(dispatch);
  }

  /**
   * @copydoc CaptorInterface::set_capacity
   */
  inline void set_capacity_impl(const size_type capacity)
  {
    CaptorInterfaceType::capacity_ = capacity;
  }

  /**
   * @copydoc CaptorInterface::get_capacity
   */
  inline size_type get_capacity_impl() const
  {
    return CaptorInterfaceType::capacity_;
  }

  /**
   * @copydoc CaptorInterface::capture
   */
  template<typename OutputDispatchIteratorT, typename CaptureRangeT>
  inline State capture_impl(OutputDispatchIteratorT&& output,
                            CaptureRangeT&& range,
                            const std::chrono::system_clock::time_point timeout = std::chrono::system_clock::time_point::max())
  {
    return derived()->capture_policy_impl(std::forward<OutputDispatchIteratorT>(output),
                                          std::forward<CaptureRangeT>(range));
  }

  using CaptorInterfaceType = CaptorInterface<Captor<CaptorT, NoLock>>;
  friend CaptorInterfaceType;

  FLOW_IMPLEMENT_CRTP_BASE(CaptorT);

protected:
  using CaptorInterfaceType::queue_;
};

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_CAPTOR_NOLOCK_HPP
