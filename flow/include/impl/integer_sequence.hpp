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

/**
 * For reference, see https://en.cppreference.com/w/cpp/utility/integer_sequence
 */
#ifndef FLOW_IMPL_INTEGER_SEQUENCE_H
#define FLOW_IMPL_INTEGER_SEQUENCE_H
#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <type_traits>
#include <utility>

namespace flow
{

/**
 * @brief Represents a compile-time sequence of integers
 * @tparam IntT  integer element type
 * @tparam Ns...  pack of integer values of type \p IntT
 */
template <typename IntT, IntT... Ns> struct integer_sequence
{
  static_assert(std::is_integral<IntT>(), "'IntT' should be an integral type");

  /**
   * @brief Returns the number of elements in <code>IntT... Ns</code>
   */
  static constexpr size_t size() noexcept { return sizeof...(Ns); }
};

namespace detail
{
/**
 * @brief Helper which simplifies creation of <code>sequence</code>
 */
template <typename IntT, size_t... Ns> struct make_sequence;

/**
 * @copydoc make_sequence
 */
template <typename IntT, size_t Index, size_t... Ns> struct make_sequence<IntT, Index, Ns...>
{
  using type = typename make_sequence<IntT, Index - 1, Index - 1, Ns...>::type;
};

/**
 * @copydoc make_sequence
 */
template <typename IntT, size_t... Ns> struct make_sequence<IntT, 0, Ns...>
{
  using type = integer_sequence<IntT, Ns...>;
};

}  // namespace detail

/**
 * @brief Integer sequence where the integer type is <code>std::size_t</code>
 * @tparam IntT  integer element type
 * @tparam N  number of indices in sequence, i.e. <code>[0, 1, ..., N-2, N-1]</code>
 */
template <typename IntT, size_t N> using make_integer_sequence = typename detail::make_sequence<IntT, N>::type;

/**
 * @brief Integer sequence where the integer type is <code>size_t</code>
 * @tparam Ns...  pack of integer values of type <code>std::size_t</code>
 */
template <size_t... Ns> using index_sequence = integer_sequence<size_t, Ns...>;

/**
 * @brief Makes an integer sequence where the integer type is <code>size_t</code>
 * @tparam N  number of indices in sequence, i.e. <code>[0, 1, ..., N-2, N-1]</code>
 */
template <size_t N> using make_index_sequence = make_integer_sequence<size_t, N>;

/**
 * @brief Makes an index sequence type with elements for each type in <code>T...</code>
 * @param TPack...  pack of arbitrary types
 */
template <class... TPack> using index_sequence_for = make_integer_sequence<size_t, sizeof...(TPack)>;

}  // namespace flow

#endif  // DOXYGEN_SKIP
#endif  // FLOW_IMPL_INTEGER_SEQUENCE_H
