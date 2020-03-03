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

template<typename DispatchT, typename AllocatorT>
DispatchQueue<DispatchT, AllocatorT>::DispatchQueue(const AllocatorT& alloc) :
  queue_{alloc}
{}

template<typename DispatchT, typename AllocatorT>
typename DispatchQueue<DispatchT, AllocatorT>::
size_type DispatchQueue<DispatchT, AllocatorT>::size() const
{
  return queue_.size();
}


template<typename DispatchT, typename AllocatorT>
bool DispatchQueue<DispatchT, AllocatorT>::empty() const
{
  return !size();
}


template<typename DispatchT, typename AllocatorT>
template<typename... DispatchConstructorArgTs>
void DispatchQueue<DispatchT, AllocatorT>::insert(DispatchConstructorArgTs&&... dispatch_args)
{
  DispatchT dispatch{std::forward<DispatchConstructorArgTs>(dispatch_args)...};

  // If data to add is ordered with respect to current queue,
  // add to back (as newest element)
  if (queue_.empty() or (queue_.back().stamp() < dispatch.stamp()))
  {
    queue_.emplace_back(std::move(dispatch));
    return;
  }

  // Find next best placement
  auto qitr = queue_.end();
  while((--qitr)->stamp() > dispatch.stamp())
  {
    if (qitr == queue_.begin())
    {
      queue_.emplace_front(std::move(dispatch));
      return;
    }
  }
  queue_.emplace(std::next(qitr), std::move(dispatch));
}


template<typename DispatchT, typename AllocatorT>
DispatchT DispatchQueue<DispatchT, AllocatorT>::pop()
{
  const auto retval = std::move(queue_.front());
  queue_.pop_front();
  return retval;
}


template<typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::clear()
{
  queue_.clear();
}


template<typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::remove_before(const stamp_type& t)
{
  while (!queue_.empty() and queue_.front().stamp() < t)
  {
    queue_.pop_front();
  }
}


template<typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::remove_at_before(const stamp_type& t)
{
  while (!queue_.empty() and queue_.front().stamp() <= t)
  {
    queue_.pop_front();
  }
}


template<typename DispatchT, typename AllocatorT>
void DispatchQueue<DispatchT, AllocatorT>::shrink_to_fit(size_type n)
{
  while (queue_.size() > n)
  {
    queue_.pop_front();
  }
}


template<typename DispatchT, typename AllocatorT>
typename
DispatchQueue<DispatchT, AllocatorT>::const_iterator
DispatchQueue<DispatchT, AllocatorT>::seek_before(const stamp_type& stamp, const_iterator start) const
{
  if (start == queue_.end())
  {
    return start;
  }

  // Find iterator one before OR exactly at limiting stamp
  while (start != queue_.end() and start->stamp() <= stamp)
  {
    ++start;
  }
  return std::prev(start);
}


template<typename DispatchT, typename AllocatorT>
typename
DispatchQueue<DispatchT, AllocatorT>::const_iterator
DispatchQueue<DispatchT, AllocatorT>::seek_after(const stamp_type& stamp, const_iterator start) const
{
  // Find iterator one past limiting stamp
  while (start != queue_.end() and start->stamp() <= stamp)
  {
    ++start;
  }
  return start;
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
