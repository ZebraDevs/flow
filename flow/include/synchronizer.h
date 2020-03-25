/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file synchronizer.h
 */
#ifndef FLOW_SYNCHRONIZER_H
#define FLOW_SYNCHRONIZER_H

// C++ Standard Library
#include <chrono>
#include <memory>
#include <ostream>
#include <tuple>

// Flow
#include <flow/captor.h>
#include <flow/drivers.h>
#include <flow/followers.h>

namespace flow
{

// Forward decl
template<typename... CaptorTs>
class Synchronizer;


/**
 * @brief Synchronizer type traits helper
 * @tparam SynchronizerT  synchronizer object type
 */
template<typename SynchronizerT>
struct SynchronizerTraits
#ifndef DOXYGEN_SKIP
;
template<typename... CaptorTs>
struct SynchronizerTraits<Synchronizer<CaptorTs...>>
#endif  // DOXYGEN_SKIP
{
  /// Synchronizer stamp type
  using stamp_type = decltype(detail::check_stamp_type<CaptorTs...>());

  /// Synchronizer captor types
  using CaptorTypes = std::tuple<CaptorTs...>;
};


/// Convenience using template to access Synchronizer time stamp type
template<typename SynchronizerT>
using sync_stamp_type_t = typename SynchronizerTraits<SynchronizerT>::stamp_type;


/// Convenience using template to access tuple of Synchronizer captor types
template<typename SynchronizerT>
using sync_captors_t = typename SynchronizerTraits<SynchronizerT>::CaptorTypes;


/**
 * @brief Data synchronization block
 *
 *        An Synchronizer synchronizes data across several capture buffers
 *
 * @tparam CaptorTs...  A pack of input capture buffer types. The first type in this pack
 *                      is required to fulfill the requirements of a driving capture buffer
 *                      (Driver). The remaining types must fulfill the requirements of a
 *                      following capture buffer (Follower)
 */
template<typename... CaptorTs>
class Synchronizer
{
public:
  /// Captor dispatch sequence stamp type
  using stamp_type = sync_stamp_type_t<Synchronizer>;

  /**
   * @brief Event synchronization results
   */
  struct Result
  {
    /// Captor state on exit
    State state;

    /// Driving sequencing stamp range
    CaptureRange<stamp_type> range;

    /// Default constructor
    inline Result() :
      state{State::RETRY}
    {}

    /**
     * @brief Operator overload to check if synchronization succeeded from details
     */
    inline operator bool() const
    {
      return state == State::PRIMED;
    }
  };

  /**
   * @brief Initialization constructor
   * @param latest_stamp  initial time-guard value
   */
  explicit Synchronizer(const stamp_type latest_stamp = StampTraits<stamp_type>::min());

  /**
   * @brief Copy constructor
   */
  Synchronizer(const Synchronizer&) = default;

  /**
   * @brief Move constructor
   */
  Synchronizer(Synchronizer&&) = default;

  /**
   * @brief Deconstructor
   * @note Calls <code>Synchronizer::shutdown</code>
   */
  ~Synchronizer();

  /**
   * @brief Abort active capture at and before \p t_abort
   *
   *        Call will result in captors removing buffered data according to thier
   *        specific abort policy. This method may be used to skip capture frames
   *        if input data capture does not happen within a particular timeout period.
   *
  *
   * @param captors  tuple of captors used to perform synchronization
   * @param t_abort  abort time point
   *
   * @warning  Calling this method may result in data loss
   */
  void abort(const std::tuple<CaptorTs&...>& captors, const stamp_type t_abort);

  /**
   * @brief Release all internal input capture waits
   *
   * @param captors  tuple of captors used to perform synchronization
   *
   * @note  Calling this method will reset all captor states
   */
  void reset(const std::tuple<CaptorTs&...>& captors);

  /**
   * @brief Runs event input capture
   *
   * @param captors  tuple of captors used to perform synchronization
   * @param outputs  tuple of dispath output iterators, order w.r.t <code>CaptorTs</code>
   * @param timeout  synchronization timeout for captors which require a data wait
   *
   * @return capture/synchronization details
   */
  template<typename... OutputIteratorTs>
  Result capture(const std::tuple<CaptorTs&...>& captors,
                 const std::tuple<OutputIteratorTs...> outputs,
                 const std::chrono::system_clock::time_point timeout = std::chrono::system_clock::time_point::max());

private:
  /// Sequencing stamp of the latest valid result
  stamp_type latest_stamp_;

  /**
   * @brief Output stream overload for <code>Synchronizer::Result</code> codes
   * @param[in,out] os  output stream
   * @param result  Synchronizer result object
   * @return os
   */
  friend inline std::ostream& operator<<(std::ostream& os, const Synchronizer::Result& result)
  {
    return os << "state: " << result.state << ", range: " << result.range;
  }
};

}  // namespace flow

// Flow (implementation)
#include <flow/impl/synchronizer.hpp>

#endif  // FLOW_SYNCHRONIZER_H
