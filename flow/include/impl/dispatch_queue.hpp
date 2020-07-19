/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
#define FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP

// C++ Standard Library
#include <iterator>

// Flow
#include <flow/dispatch.h>

namespace flow
{

template <typename DispatchT, typename ContainerT>
DispatchQueue<DispatchT, ContainerT>::DispatchQueue(const ContainerT& container) : container_{container}
{}

template <typename DispatchT, typename ContainerT>
typename DispatchQueue<DispatchT, ContainerT>::size_type DispatchQueue<DispatchT, ContainerT>::size() const
{
  return container_.size();
}


template <typename DispatchT, typename ContainerT> bool DispatchQueue<DispatchT, ContainerT>::empty() const
{
  return !size();
}


template <typename DispatchT, typename ContainerT>
template <typename... DispatchConstructorArgTs>
void DispatchQueue<DispatchT, ContainerT>::insert(DispatchConstructorArgTs&&... dispatch_args)
{
  DispatchT dispatch{std::forward<DispatchConstructorArgTs>(dispatch_args)...};

  // If data to add is ordered with respect to current queue,
  // add to back (as newest element)
  if (container_.empty() or (get_stamp(container_.back()) < get_stamp(dispatch)))
  {
    container_.emplace_back(std::move(dispatch));
    return;
  }

  // Find next best placement
  auto qitr = container_.end();
  while (get_stamp(*(--qitr)) > get_stamp(dispatch))
  {
    if (qitr == container_.begin())
    {
      container_.emplace_front(std::move(dispatch));
      return;
    }
  }

  // Insert only if this element does not duplicate an existing element
  if (get_stamp(*qitr) != get_stamp(dispatch))
  {
    container_.emplace(std::next(qitr), std::move(dispatch));
  }
}


template <typename DispatchT, typename ContainerT> DispatchT DispatchQueue<DispatchT, ContainerT>::pop()
{
  const auto retval = std::move(container_.front());
  container_.pop_front();
  return retval;
}


template <typename DispatchT, typename ContainerT> void DispatchQueue<DispatchT, ContainerT>::clear()
{
  container_.clear();
}


template <typename DispatchT, typename ContainerT>
void DispatchQueue<DispatchT, ContainerT>::remove_before(const stamp_type& t)
{
  while (!container_.empty() and get_stamp(container_.front()) < t)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT>
void DispatchQueue<DispatchT, ContainerT>::remove_at_before(const stamp_type& t)
{
  while (!container_.empty() and get_stamp(container_.front()) <= t)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT>
void DispatchQueue<DispatchT, ContainerT>::shrink_to_fit(const size_type n)
{
  while (container_.size() > n)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT>
const ContainerT& DispatchQueue<DispatchT, ContainerT>::get_container() const noexcept
{
  return container_.get_container();
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
