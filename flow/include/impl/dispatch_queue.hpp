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
template<typename IteratorType>
typename
DispatchQueue<DispatchT, AllocatorT>::size_type
DispatchQueue<DispatchT, AllocatorT>::advance(IteratorType& itr, const stamp_type& target) const
{
  size_type count_before{0};

  while (itr->stamp() < target)
  {
    ++count_before;
    if (++itr == queue_.cend())
    {
      return count_before;
    }
  }
  return count_before;
}


template<typename DispatchT, typename AllocatorT>
template<typename OutputDispatchIteratorT>
std::tuple<typename DispatchQueue<DispatchT, AllocatorT>::size_type,
           typename DispatchQueue<DispatchT, AllocatorT>::size_type,
           typename DispatchQueue<DispatchT, AllocatorT>::stamp_type>
DispatchQueue<DispatchT, AllocatorT>::capture_around(OutputDispatchIteratorT output,
                                                     const CaptureRange<stamp_type>& range,
                                                     size_type n_before,
                                                     size_type m_after) const
{
  // Abort when queue is empty
  if (queue_.empty())
  {
    return std::make_tuple(0, 0, StampTraits<stamp_type>::min());
  }

  // Scroll to first valid data element
  auto itr = queue_.cbegin();
  const size_type count_before = advance(itr, range.lower_stamp);

  // Check if there are enough elements available before range.lower_stamp
  if (n_before > count_before)
  {
    return std::make_tuple(count_before, 0, StampTraits<stamp_type>::min());
  }

  // Rewind
  itr = std::prev(itr, n_before);

  // Get first stamp
  const stamp_type first_stamp = itr->stamp();

  // Add elements before range.lower_stamp
  while (n_before--)
  {
    *(output++) = *(itr++);
  }

  // Add element until range.upper_stamp
  size_type count_middle{count_before};
  while (itr != queue_.cend() and
         itr->stamp() < range.upper_stamp)
  {
    *(output++) = *(itr++);
    ++count_middle;
  }

  // Check if there are enough elements available after range.upper_stamp
  const size_type count_after = queue_.size() - count_middle;
  if (m_after > count_after)
  {
    return std::make_tuple(count_before, count_after, first_stamp);
  }

  // Add element after range.upper_stamp
  while (m_after--)
  {
    *(output++) = *(itr++);
  }

  return std::make_tuple(count_before, count_after, first_stamp);
}

}  // namespace flow

#endif  // FLOW_CAPTURE_IMPL_DISPATCH_QUEUE_HPP
