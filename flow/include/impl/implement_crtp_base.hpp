/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file implement_crtp_base.hpp
 * @brief Defines a macro for CRTP-derived objects
 */
#ifndef FLOW_IMPL_IMPLEMENT_CRTP_BASE_H
#define FLOW_IMPL_IMPLEMENT_CRTP_BASE_H

/**
 * @brief Macro used to implement CRTP-derived type helpers in a uniform way
 * @param DerivedT  CRTP-derived type
 */
#define FLOW_IMPLEMENT_CRTP_BASE(DerivedT)\
inline DerivedT* derived()\
{\
  return static_cast<DerivedT*>(this);\
}\
inline const DerivedT* derived() const\
{\
  return static_cast<const DerivedT*>(this);\
}

#endif  // FLOW_IMPL_IMPLEMENT_CRTP_BASE_H
