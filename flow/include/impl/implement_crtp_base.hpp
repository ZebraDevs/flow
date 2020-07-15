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

#ifndef FLOW_IMPL_IMPLEMENT_CRTP_BASE_H
#define FLOW_IMPL_IMPLEMENT_CRTP_BASE_H

/**
 * @brief Macro used to implement CRTP-derived type helpers in a uniform way
 * @param DerivedT  CRTP-derived type
 */
#define FLOW_IMPLEMENT_CRTP_BASE(DerivedT)                                                                             \
  inline DerivedT* derived() { return static_cast<DerivedT*>(this); }                                                  \
  inline const DerivedT* derived() const { return static_cast<const DerivedT*>(this); }

#endif  // FLOW_IMPL_IMPLEMENT_CRTP_BASE_H
