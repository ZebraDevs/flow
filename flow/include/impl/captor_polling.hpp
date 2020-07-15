// Copyright (C) 2020, Fetch Robotics Inc.
//
// This file is part of Flow.
//
// Flow is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Flow is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Flow.  If not, see <https://www.gnu.org/licenses/>.

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
 * @note Implements a simple locking policy using a <code>BasicLockableT</code> which
 *       protects data input/output, meant for polling with <code>Captor::capture</code>.
 */
template <typename CaptorT, typename BasicLockableT>
class Captor<CaptorT, PollingLock<BasicLockableT>>
    : public CaptorInterface<Captor<CaptorT, PollingLock<BasicLockableT>>>
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
  Captor(const DispatchAllocatorType& alloc) : CaptorInterfaceType{0UL, alloc} {}

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

    // Run reset behavior specific to this captor
    derived()->reset_policy_impl();

    // Remove all data
    CaptorInterfaceType::queue_.clear();
  }

  /**
   * @copydoc CaptorInterface::abort
   */
  inline void abort_impl(const stamp_type& t_abort)
  {
    BasicLockableT lock{queue_mutex_};

    // Run abort behavior specific to this captor
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
  template <typename... DispatchConstructorArgTs> inline void inject_impl(DispatchConstructorArgTs&&... dispatch_args)
  {
    BasicLockableT lock{queue_mutex_};
    CaptorInterfaceType::insert_and_limit(std::forward<DispatchConstructorArgTs>(dispatch_args)...);
  }

  /**
   * @copydoc CaptorInterface::insert
   */
  template <typename FirstForwardDispatchIteratorT, typename LastForwardDispatchIteratorT>
  inline void insert_impl(FirstForwardDispatchIteratorT first, LastForwardDispatchIteratorT last)
  {
    BasicLockableT lock{queue_mutex_};
    std::for_each(
      first, last, [this](const DispatchType& dispatch) { CaptorInterfaceType::insert_and_limit(dispatch); });
  }

  /**
   * @copydoc CaptorInterface::remove
   */
  inline void remove_impl(const stamp_type& t_remove)
  {
    BasicLockableT lock{queue_mutex_};

    // Remove all data before this time
    CaptorInterfaceType::queue_.remove_before(t_remove);
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
    return queue_.empty() ? CaptureRange<stamp_type>{}
                          : CaptureRange<stamp_type>{queue_.oldest_stamp(), queue_.newest_stamp()};
  }

  /**
   * @copydoc CaptorInterface::capture
   */
  template <typename OutputDispatchIteratorT, typename CaptureRangeT>
  inline State capture_impl(OutputDispatchIteratorT&& output, CaptureRangeT&& range)
  {
    BasicLockableT lock{queue_mutex_};
    return derived()->capture_policy_impl(
      std::forward<OutputDispatchIteratorT>(output), std::forward<CaptureRangeT>(range));
  }

  /**
   * @copydoc CaptorInterface::dry_capture
   */
  template <typename CaptureRangeT> inline State dry_capture_impl(CaptureRangeT&& range) const
  {
    BasicLockableT lock{queue_mutex_};
    return derived()->dry_capture_policy_impl(std::forward<CaptureRangeT>(range));
  }

  /**
   * @copydoc CaptorInterface::capture
   */
  template <typename InpectCallbackT> void inspect_impl(InpectCallbackT&& inspect_dispatch_cb) const
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
