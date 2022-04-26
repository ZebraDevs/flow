/**
 * @copyright 2020-present Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @brief Defines a utility similar to <code>std::apply</code> (C++14)
 *
 *        For reference, see
 *        <a href="http://en.cppreference.com/w/cpp/utility/apply">here</a>
 */
#ifndef FLOW_UTILITY_APPLY_HPP
#define FLOW_UTILITY_APPLY_HPP

// C++ Standard Library
#include <cstdint>
#include <tuple>
#include <utility>

// Flow
#include <flow/utility/integer_sequence.hpp>
#include <flow/utility/static_assert.hpp>

namespace flow
{

#ifndef DOXYGEN_SKIP
namespace detail
{

/**
 * @brief Implementation for flow::apply
 */
template <typename UnaryInvocableT, typename ArgTupleT, std::size_t... IntPack>
constexpr decltype(auto) apply(UnaryInvocableT&& fn, ArgTupleT&& targs, index_sequence<IntPack...>)
{
  FLOW_STATIC_ASSERT(
    std::tuple_size<std::remove_reference_t<ArgTupleT>>::value == sizeof...(IntPack), "ArgTupleT size is invalid");
  return fn(std::get<IntPack>(std::forward<ArgTupleT>(targs))...);
}

}  // namespace detail
#endif  // DOXYGEN_SKIP

/**
 * @copydoc flow::apply
 * @return return value of <code>fn</code> when passed arguments from <code>targs</code>
 * @note Participates in overload resolution when \p ReturnT is non-<code>void</code>
 */
template <typename UnaryInvocableT, typename ArgTupleT>
constexpr decltype(auto) apply(UnaryInvocableT&& fn, ArgTupleT&& targs)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference<ArgTupleT>::type>::value;
  return detail::apply(std::forward<UnaryInvocableT>(fn), std::forward<ArgTupleT>(targs), make_index_sequence<N>{});
}

#ifndef DOXYGEN_SKIP
namespace detail
{

/**
 * @brief Calls \c std::apply on a tuple formed of elements at each \c TupleTs at \c ArgPosition
 */
template <std::size_t ArgPosition> struct NAryAdaptor
{
  template <typename NAryInvocableT, typename... ArgTuples>
  inline static decltype(auto) exec(NAryInvocableT&& fn, ArgTuples&&... tups)
  {
    return ::flow::apply(
      std::forward<NAryInvocableT>(fn), std::forward_as_tuple(std::get<ArgPosition>(std::forward<ArgTuples>(tups))...));
  }
};

/**
 * @brief Implementation for flow::apply_every
 */
template <typename NAryInvocableT, std::size_t... ArgPositions, typename... ArgTupleTs>
constexpr void apply_every(NAryInvocableT&& fn, index_sequence<ArgPositions...>, ArgTupleTs&&... targs)
{
  std::initializer_list<int>{
    0,
    (NAryAdaptor<ArgPositions>::template exec(std::forward<NAryInvocableT>(fn), std::forward<ArgTupleTs>(targs)...),
     0)...};
}


/**
 * @brief Implementation for flow::apply_every_r
 */
template <typename NAryInvocableT, std::size_t... ArgPositions, typename... ArgTupleTs>
constexpr auto apply_every_r(NAryInvocableT&& fn, index_sequence<ArgPositions...>, ArgTupleTs&&... targs)
{
  return std::make_tuple(
    NAryAdaptor<ArgPositions>::template exec(std::forward<NAryInvocableT>(fn), std::forward<ArgTupleTs>(targs)...)...);
}


/**
 * @brief Implementation for flow::apply_each
 */
template <typename UnaryInvocableTupleT, typename ArgTupleT, std::size_t... IntPack>
constexpr void apply_each(UnaryInvocableTupleT&& fns, ArgTupleT&& targs, index_sequence<IntPack...>)
{
  std::initializer_list<int>{0, (std::get<IntPack>(fns)(std::get<IntPack>(targs)), 0)...};
}


}  // namespace detail
#endif  // DOXYGEN_SKIP

/**
 * @brief Calls function with with tuple (e.g <code>std::tuple</code>) as argument list
 *
 * @tparam ReturnT     function return type
 * @tparam UnaryInvocableT  (deduced) invocable type:
 *                     - function pointer
 *                     - functor object with <code>Object::operator()</code>
 *                     - lambda function expression
 * @tparam TupleT  (deduced) tuple-like type with element types order according to function argument list
 *
 * @param fn     function pointer or functor object type
 * @param targs  tuple with arguments to pass to <code>f</code>
 */
template <typename ReturnT, typename UnaryInvocableT, typename ArgTupleT>
constexpr typename std::enable_if<std::is_void<ReturnT>::value>::type apply(UnaryInvocableT&& fn, ArgTupleT&& targs)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference<ArgTupleT>::type>::value;
  detail::apply<ReturnT>(std::forward<UnaryInvocableT>(fn), std::forward<ArgTupleT>(targs), make_index_sequence<N>{});
}


/**
 * @brief Calls a N-ary function on each element of N tuples (e.g <code>std::tuple</code>)
 *
 * @tparam NAryInvocableT  (deduced) n-ary invocable type:
 *                              - function pointer
 *                              - functor object with <code>Object::operator()</code>
 *                              - lambda function expression
 * @tparam ArgTuples  (deduced) tuple-like type with element types order according to function argument list
 *
 * @param fn    function pointer or functor object
 * @param targs  tuple with arguments to pass to <code>f</code>
 */
template <typename NAryInvocableT, typename... ArgTuples>
constexpr void apply_every(NAryInvocableT&& fn, ArgTuples&&... targs)
{
  using ArgTuple0 = std::remove_reference_t<std::tuple_element_t<0, std::tuple<ArgTuples...>>>;
  constexpr auto N = std::tuple_size<ArgTuple0>::value;
  detail::apply_every(std::forward<NAryInvocableT>(fn), make_index_sequence<N>{}, std::forward<ArgTuples>(targs)...);
}


/**
 * @brief Calls a N-ary function with a returned value on each element of N tuples (e.g <code>std::tuple</code>)
 *
 * @tparam NAryInvocableT  (deduced) n-ary invocable type:
 *                              - function pointer
 *                              - functor object with <code>Object::operator()</code>
 *                              - lambda function expression
 * @tparam ArgTuples  (deduced) tuple-like type with element types order according to function argument list
 *
 * @param fn    function pointer or functor object
 * @param targs  tuple with arguments to pass to <code>f</code>
 *
 * @return tuple of return values
 */
template <typename NAryInvocableT, typename... ArgTuples>
constexpr auto apply_every_r(NAryInvocableT&& fn, ArgTuples&&... targs)
{
  using ArgTuple0 = std::remove_reference_t<std::tuple_element_t<0, std::tuple<ArgTuples...>>>;
  constexpr auto N = std::tuple_size<ArgTuple0>::value;
  return detail::apply_every_r(
    std::forward<NAryInvocableT>(fn), make_index_sequence<N>{}, std::forward<ArgTuples>(targs)...);
}


/**
 * @brief Calls a unique function on each element of a tuple (e.g <code>std::tuple</code>)
 *
 * @tparam UnaryInvocableTupleT  (deduced) binary invocable type:
 *                              - function pointer
 *                              - functor object with <code>Object::operator()</code>
 *                              - lambda function expression
 * @tparam ArgTupleT  (deduced) tuple-like type with element types order according to function argument list
 *
 * @param fns    tuple of function pointers or functor objects
 * @param targs  tuple with arguments to pass to <code>f</code>
 */
template <typename UnaryInvocableTupleT, typename ArgTupleT>
constexpr void apply_each(UnaryInvocableTupleT&& fns, ArgTupleT&& targs)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference<ArgTupleT>::type>::value;
  detail::apply_each(std::forward<UnaryInvocableTupleT>(fns), std::forward<ArgTupleT>(targs), make_index_sequence<N>{});
}

}  // namespace flow

#endif  // FLOW_UTILITY_APPLY_HPP
