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

#ifndef DOXYGEN_SKIP

// C++ Standard Library
#include <string>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow/dispatch.h>

using namespace flow;


TEST(Dispatch, GetData)
{
  Dispatch<int, std::string> d{2, "test-value"};

  EXPECT_EQ(get_value(d), "test-value");
}


TEST(Dispatch, GetStamp)
{
  Dispatch<int, std::string> d{3, "test-value"};

  EXPECT_EQ(get_stamp(d), 3);
}

#endif  // DOXYGEN_SKIP
