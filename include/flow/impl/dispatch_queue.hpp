/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_IMPL_DISPATCH_QUEUE_HPP
#define FLOW_IMPL_DISPATCH_QUEUE_HPP

// C++ Standard Library
#include <algorithm>
#include <iterator>

namespace flow
{

template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::DispatchQueue(const ContainerT& container) :
    container_{container}
{}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
typename DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::size_type
DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::size() const
{
  return container_.size();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
bool DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::empty() const
{
  return !size();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
template <typename OutputDispatchIteratorT>
OutputDispatchIteratorT DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::copy(
  OutputDispatchIteratorT output,
  const ExtractionRange& extraction_range) const
{
  return std::copy(
    std::next(container_.begin(), extraction_range.first),
    std::next(container_.begin(), extraction_range.last),
    output);
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
template <typename OutputDispatchIteratorT>
OutputDispatchIteratorT DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::move(
  OutputDispatchIteratorT output,
  const ExtractionRange& extraction_range)
{
  return std::move(
    std::next(container_.begin(), extraction_range.first),
    std::next(container_.begin(), extraction_range.last),
    output);
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
template <typename... DispatchConstructorArgTs>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::insert(
  DispatchConstructorArgTs&&... dispatch_args)
{
  DispatchT dispatch{std::forward<DispatchConstructorArgTs>(dispatch_args)...};

  // If data to add is ordered with respect to current queue,
  // add to back (as newest element)
  if (container_.empty() or (AccessStamp::get(container_.back()) < AccessStamp::get(dispatch)))
  {
    container_.emplace_back(std::move(dispatch));
    return;
  }

  // Find next best placement
  auto qitr = container_.end();
  while (AccessStamp::get(*(--qitr)) > AccessStamp::get(dispatch))
  {
    if (qitr == container_.begin())
    {
      container_.emplace_front(std::move(dispatch));
      return;
    }
  }

  // Insert only if this element does not duplicate an existing element
  if (AccessStamp::get(*qitr) != AccessStamp::get(dispatch))
  {
    container_.emplace(std::next(qitr), std::move(dispatch));
  }
}

template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
typename DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::const_iterator
DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::before(stamp_const_arg_type stamp) const
{
  const auto after_itr = std::find_if(container_.begin(), container_.end(), [stamp](const DispatchT& dispatch) {
    return AccessStamp::get(dispatch) >= stamp;
  });

  return after_itr == this->end() ? after_itr : std::prev(after_itr);
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
typename DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::const_reverse_iterator
DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::rbefore(stamp_const_arg_type stamp) const
{
  return std::find_if(container_.rbegin(), container_.rend(), [stamp](const DispatchT& dispatch) {
    return AccessStamp::get(dispatch) < stamp;
  });
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
DispatchT& DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::top()
{
  return container_.front();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
const DispatchT& DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::top() const
{
  return container_.front();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::pop()
{
  container_.pop_front();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::clear()
{
  container_.clear();
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::remove_before(stamp_const_arg_type t)
{
  while (!container_.empty() and AccessStamp::get(container_.front()) < t)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::remove_at_before(stamp_const_arg_type t)
{
  while (!container_.empty() and AccessStamp::get(container_.front()) <= t)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::remove_first_n(const size_type n)
{
  container_.erase(container_.begin(), std::next(container_.begin(), n));
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
void DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::shrink_to_fit(const size_type n)
{
  while (container_.size() > n)
  {
    container_.pop_front();
  }
}


template <typename DispatchT, typename ContainerT, typename AccessStampT, typename AccessValueT>
const ContainerT& DispatchQueue<DispatchT, ContainerT, AccessStampT, AccessValueT>::get_container() const noexcept
{
  return container_.get_container();
}

}  // namespace flow

#endif  // FLOW_IMPL_DISPATCH_QUEUE_HPP
