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

#ifndef FLOW_CAPTOR_DISPATCH_H
#define FLOW_CAPTOR_DISPATCH_H

// C++ Standard Library
#include <chrono>
#include <limits>
#include <ostream>
#include <type_traits>
#include <utility>

namespace flow
{

/**
 * @brief Helper struct used to specify stamp attributes
 *
 *        Must be specialized for non-integral sequence stamp types with the same fields:
 *        - <code>min</code> : the minimum value of \p StampT
 *        - <code>max</code> : the maximum value of \p StampT
 *        - <code>offset_type</code> : associated offset/duration type
 *
 * @tparam StampT  sequence stamp type
 */
template <typename StampT> struct StampTraits
{
  static_assert(std::is_fundamental<StampT>(), "'StampT' must be fundemental type");

  /// Stamp type
  using stamp_type = StampT;

  /// Associated duration/offset type
  using offset_type = typename std::make_signed<StampT>::type;

  /// Returns minimum stamp value
  static constexpr StampT min() { return std::numeric_limits<StampT>::min(); };

  /// Returns maximum stamp value
  static constexpr StampT max() { return std::numeric_limits<StampT>::max(); };
};


/**
 * @brief Helper struct used to specify stamp attributes for <chrono> time types
 *
 * @tparam ClockT     clock on which this time point is measured
 * @tparam DurationT  <code>std::chrono::duration</code> type used to measure the time since epoch
 */
template <typename ClockT, typename DurationT> struct StampTraits<std::chrono::time_point<ClockT, DurationT>>
{
  /// Stamp type
  using stamp_type = std::chrono::time_point<ClockT, DurationT>;

  /// Associated duration/offset type
  using offset_type = DurationT;

  /// Returns minimum stamp value
  static constexpr stamp_type min() { return stamp_type::min(); };

  /// Returns maximum stamp value
  static constexpr stamp_type max() { return stamp_type::max(); };
};


/**
 * @brief Dispatch data wrapper
 *
 *        Custom "dispatching" objects may be used with this library in lieu of
 *        the standard <code>Dispatch</code> template provided in this file
 * \n
 *        Custom dispatch types must also specialize the <code>DispatchTraits</code> helper
 *        struct, which is used to provide core <code>stamp_type</code> and <code>value_type</code>
 *        to other dependent objects throughout the library.
 *
 * @tparam StampT  sequencing stamp type; used for data ordering (e.g. time, sequence number, etc.)
 * @tparam ValueT  data value type; must be copyable
 */
template <typename StampT, typename ValueT> class Dispatch
{
public:
  Dispatch() = default;

  /**
   * @brief Dispatch value constructor (move enabled)
   */
  template <typename... ValueArgsTs>
  explicit Dispatch(const StampT& _stamp, ValueArgsTs&&... _value_args) :
      stamp{_stamp},
      value{std::forward<ValueArgsTs>(_value_args)...}
  {}

  /// Sequencing stamp associated with data
  StampT stamp;

  /// Data element
  ValueT value;
};


/**
 * @brief Output stream overload for <code>Dispatch</code> codes
 * @param[in,out] os  output stream
 * @param dispatch  dispatch object
 * @return os
 */
template <typename StampT, typename ValueT>
inline std::ostream& operator<<(std::ostream& os, const Dispatch<StampT, ValueT>& dispatch)
{
  return os << "stamp: " << dispatch.stamp << "\nvalue: " << dispatch.value;
}


/**
 * @brief Dispatch type traits struct
 */
template <typename DispatchT> struct DispatchTraits;


/**
 * @copydoc DispatchTraits
 *
 * @note partial specialization for Dispatch
 */
template <typename StampT, typename ValueT> struct DispatchTraits<Dispatch<StampT, ValueT>>
{
  /// Dispatch stamp type
  using stamp_type = StampT;

  /// Dispatch data type
  using value_type = ValueT;
};


/**
 * @copydoc DispatchTraits
 *
 * @note partial specialization for <code>::std::pair</code>
 */
template <typename StampT, typename ValueT> struct DispatchTraits<::std::pair<StampT, ValueT>>
{
  /// Dispatch stamp type
  using stamp_type = StampT;

  /// Dispatch data type
  using value_type = ValueT;
};


/**
 * @brief Dispatch access helper
 */
template <typename DispatchT> struct DispatchAccess;


/**
 * @copydoc DispatchAccess
 *
 * @note partial specialization for Dispatch
 */
template <typename StampT, typename ValueT> struct DispatchAccess<Dispatch<StampT, ValueT>>
{
  /// Selects type of lesser size to use when returning values
  template <typename T> using return_t = std::conditional_t<(sizeof(T) <= sizeof(T&)), T, const T&>;

  static constexpr return_t<StampT> stamp(const Dispatch<StampT, ValueT>& dispatch) { return dispatch.stamp; }

  static constexpr return_t<ValueT> value(const Dispatch<StampT, ValueT>& dispatch) { return dispatch.value; }
};


/**
 * @copydoc DispatchAccess
 *
 * @note partial specialization for <code>::std::pair</code>
 */
template <typename StampT, typename ValueT> struct DispatchAccess<::std::pair<StampT, ValueT>>
{
  /// Selects type of lesser size to use when returning values
  template <typename T> using return_t = std::conditional_t<(sizeof(T) <= sizeof(T&)), T, const T&>;

  static constexpr return_t<StampT> stamp(const ::std::pair<StampT, ValueT>& dispatch) { return dispatch.first; }

  static constexpr return_t<ValueT> value(const ::std::pair<StampT, ValueT>& dispatch) { return dispatch.second; }
};


/**
 * @brief Accesses Dispatch stamp through appropriate accessors
 */
template <typename DispatchT> constexpr auto get_stamp(DispatchT&& dispatch)
{
  using D = std::remove_const_t<std::remove_reference_t<DispatchT>>;
  return DispatchAccess<D>::stamp(std::forward<DispatchT>(dispatch));
}


/**
 * @brief Accesses Dispatch value through appropriate accessors
 */
template <typename DispatchT> constexpr auto get_value(DispatchT&& dispatch)
{
  using D = std::remove_const_t<std::remove_reference_t<DispatchT>>;
  return DispatchAccess<D>::value(std::forward<DispatchT>(dispatch));
}


/**
 * @brief Data capture/sequencing information
 *
 * @tparam StampT  sequence stamp type
 */
template <typename StampT> struct CaptureRange
{
  /// Target sequence stamp produced from captured data associated with oldest captured element
  StampT lower_stamp;

  /// Target sequence stamp produced from captured data associated with newest captured element
  StampT upper_stamp;

  /**
   * @brief Sequencing range constructor
   *
   * @param _lower_stamp  lower sequencing stamp bound
   * @param _upper_stamp  upper sequencing stamp bound
   */
  CaptureRange(
    const StampT _lower_stamp = StampTraits<StampT>::max(),
    const StampT _upper_stamp = StampTraits<StampT>::min()) :
      lower_stamp{_lower_stamp},
      upper_stamp{_upper_stamp}
  {}

  /**
   * @brief Checks if capture stamp range is valid
   *
   * @retval true  if upper_stamp >= lower_stamp
   * @retval false  otherwise
   */
  inline bool valid() const { return upper_stamp >= lower_stamp; }

  /**
   * @copydoc CaptureRange::valid
   */
  inline operator bool() const { return valid(); }
};


/**
 * @brief Output stream overload for <code>CaptureRange</code> codes
 *
 * @param[in,out] os  output stream
 * @param range  capture stamp range
 * @return os
 */
template <typename StampT> inline std::ostream& operator<<(std::ostream& os, const CaptureRange<StampT>& range)
{
  return os << "lower_stamp: " << range.lower_stamp << ", upper_stamp: " << range.upper_stamp;
}

}  // namespace flow

#endif  // FLOW_CAPTOR_DISPATCH_H
