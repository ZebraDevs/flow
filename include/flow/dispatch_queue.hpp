/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 */
#ifndef FLOW_DISPATCH_QUEUE_HPP
#define FLOW_DISPATCH_QUEUE_HPP

// C++ Standard Library
#include <type_traits>
#include <utility>

// Flow
#include <flow/dispatch.hpp>

namespace flow
{

/**
 * @brief Represents a range of elements
 *
 * Range is iterator-like range. When <code>first == last</code>, an "empty" range is represented
 */
struct ExtractionRange
{
  /// Position of first element
  std::size_t first = 0UL;

  /// Position of one-past-last element
  std::size_t last = 0UL;

  /**
   * @brief Checks if element range is non-empty
   *
   * @retval true  if non-empty
   * @retval false if empty
   */
  constexpr bool valid() const { return first < last; }

  /**
   * @copydoc ExtractionRange::valid
   */
  constexpr operator bool() const { return ExtractionRange::valid(); }

  ExtractionRange() = default;
  ExtractionRange(const std::size_t _first, const std::size_t _last) : first{_first}, last{_last} {}
};

/**
 * @brief Default implementation for accessing dispatch stamps
 */
struct DefaultStampAccess
{
  template <typename DispatchT> static auto get(const DispatchT& dispatch) { return get_stamp(dispatch); }
};

/**
 * @brief Default implementation for accessing dispatch values
 */
struct DefaultValueAccess
{
  template <typename DispatchT> static auto get(const DispatchT& dispatch) { return get_value(dispatch); }
};

/**
 * @brief Dispatch queuing data structure
 *
 * FILO-type queue which orders data by sequence stamp, from oldest to newest. Provides
 * useful methods for extracting data within stamped/counted ranges
 * \n
 * This template provides an interface wrapper around a specifiable container implementation.
 *
 * @tparam DispatchT  data dipatch type
 * @tparam ContainerT  underlying <code>DispatchT</code> container timplementation
 * @tparam AccessStampT  custom stamp access
 * @tparam AccessValueT  custom value access
 */
template <
  typename DispatchT,
  typename ContainerT,
  typename AccessStampT = DefaultStampAccess,
  typename AccessValueT = DefaultValueAccess>
class DispatchQueue
{
public:
  /// Dispatch stamp type
  using stamp_type = typename DispatchTraits<DispatchT>::stamp_type;

  /// Stamp argument type
  using stamp_const_arg_type =
    std::conditional_t<sizeof(stamp_type) <= sizeof(stamp_type&), const stamp_type, const stamp_type&>;

  /// Dispatch data value type
  using value_type = typename DispatchTraits<DispatchT>::value_type;

  /// Sizing type alias
  using size_type = typename ContainerT::size_type;

  /// Iterator type for container Dispatch elements
  using const_iterator = typename ContainerT::const_iterator;

  /// Iterator type for container Dispatch elements
  using const_reverse_iterator = typename ContainerT::const_reverse_iterator;

  /// Stamp access implementation
  using AccessStamp = AccessStampT;

  /// Value access implementation
  using AccessValue = AccessValueT;

  /**
   * @brief Default construtor
   */
  DispatchQueue() = default;

  /**
   * @brief Allocator construtor
   */
  explicit DispatchQueue(const ContainerT& container);

  /**
   * @brief Returns the number queued elements
   */
  inline size_type size() const;

  /**
   * @brief Checks if queue is empty
   *
   * @retval true  if not elements remain in queue
   * @retval false  otherwise
   */
  inline bool empty() const;

  /**
   * @brief Copies elements in \c range
   *
   * @param output  element output iterator
   * @param range  element index range
   */
  template <typename OutputDispatchIteratorT>
  inline OutputDispatchIteratorT copy(OutputDispatchIteratorT output, const ExtractionRange& extraction_range) const;

  /**
   * @brief Moves elements in \c range
   *
   * @param output  element output iterator
   * @param range  element index range
   */
  template <typename OutputDispatchIteratorT>
  inline OutputDispatchIteratorT move(OutputDispatchIteratorT output, const ExtractionRange& extraction_range);

  /**
   * @brief Returns first iterator to element before stamp
   *
   * Returns <code>end()</code> iterator if
   * @return <code>const_iterator</code> to first Dispatch resource
   */
  inline const_iterator before(stamp_const_arg_type stamp) const;

  /**
   * @brief Returns first iterator to underlying ordered data structure
   * @return <code>const_iterator</code> to first Dispatch resource
   */
  inline const_iterator begin() const { return container_.cbegin(); }

  /**
   * @brief Returns last iterator to underlying ordered data structure
   * @return <code>const_iterator</code> to one element past Dispatch resource
   */
  inline const_iterator end() const { return container_.cend(); }

  /**
   * @brief Returns first reverse-iterator to element before stamp
   *
   * Returns <code>rend()</code> iterator if element cannot be found or if queue is empty
   *
   * @return <code>const_reverse_iterator</code> to first Dispatch resource
   */
  inline const_reverse_iterator rbefore(stamp_const_arg_type stamp) const;

  /**
   * @brief Returns first iterator to reversed underlying ordered data structure
   *
   * @return <code>const_reverse_iterator</code> to first Dispatch resource
   */
  inline const_reverse_iterator rbegin() const { return container_.crbegin(); }

  /**
   * @brief Returns last iterator to reversed underlying ordered data structure
   *
   * @return <code>const_reverse_iterator</code> to one element past Dispatch resource
   */
  inline const_reverse_iterator rend() const { return container_.crend(); }

  /**
   * @brief Sequencing stamp associated with the oldest data
   *
   * @return sequencing stamp of first-queued Dispatch
   *
   * @warning Undefined behavior when <code>empty() == true</code>
   */
  inline stamp_type oldest_stamp() const { return AccessStampT::get(container_.front()); }

  /**
   * @brief Sequencing stamp associated with the newest data
   *
   * @return sequencing stamp of last-queued Dispatch
   *
   * @warning Undefined behavior when <code>empty() == true</code>
   */
  inline stamp_type newest_stamp() const { return AccessStampT::get(container_.back()); }

  /**
   * @brief Retrieves the oldest element
   */
  inline DispatchT& top();

  /**
   * @brief Retrieves the oldest element
   */
  inline const DispatchT& top() const;

  /**
   * @brief Removes the oldest element
   */
  inline void pop();

  /**
   * @brief Removes all data from queue
   */
  inline void clear();

  /**
   * @brief Removes data with stamp older than reference sequence stamp
   *
   * @param stamp  lower bound on container sequence stamp
   */
  inline void remove_before(stamp_const_arg_type t);

  /**
   * @brief Removes data with stamp older than or equal to some reference sequence stamp
   *
   * @param stamp  lower bound on container sequence stamp
   */
  inline void remove_at_before(stamp_const_arg_type t);

  /**
   * @brief Removes first \c n elements
   *
   * @param n  number of elements to remove
   */
  inline void remove_first_n(const size_type n);

  /**
   * @brief Removes oldest data until queue has less than or equal to N-elements
   *
   * @param n  lower bound on total container size
   */
  inline void shrink_to_fit(const size_type n);

  /**
   * @brief Inserts data in sequence stamp order as Dispatch
   *
   * @param dispatch_args  dispatch constructor args
   *
   * @warning elements with stamps identical to existing element stamps are not added
   */
  template <typename... DispatchConstructorArgTs> inline void insert(DispatchConstructorArgTs&&... dispatch_args);

  /**
   * @brief Returns the underlying storage container
   */
  inline const ContainerT& get_container() const noexcept;

private:
  /// Queued data dispatches
  ContainerT container_;
};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/dispatch_queue.hpp>

#endif  // FLOW_DISPATCH_QUEUE_HPP
