# Copyright (C) 2020, Fetch Robotics Inc.
#
# This file is part of Flow.
#
# Flow is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Flow is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Flow.  If not, see <https://www.gnu.org/licenses/>.

def __flow_copts(name, sanitize=False, debug_build=False, asserts_as_exceptions=False):
    '''
    Common copts used in custom macros
    '''
    copts = ["-std=c++1y", "-Werror", "-Wall", "-Wextra", "-Wno-deprecated-declarations", "-Wno-unused-parameter", "-g"]

    if debug_build or sanitize:
        # No optimizations
        copts += ["-O0"]

        # Log about this
        print("[" + name + "] being built in DEBUG mode")
    else:
        # Full optimizations; no debug asserts
        copts += ["-O3", "-DNDEBUG"]

    # AddressSanitizer options
    if sanitize:
        print("[" + name + "] being built with 'AddressSanitizer' copts")
        copts += ["-fsanitize=address"]

    return copts


def __flow_linkopts(name, sanitize=False):
    '''
    Common linkopts used in custom macros
    '''

    # AddressSanitizer options
    if sanitize:
        print("[" + name + "] being built with 'AddressSanitizer' linkopts")
        return ["-lasan"]

    return []


def flow_cc_library(name, copts=[], linkopts=[], sanitize=False, debug_build=False, **kwargs):
    '''
    A wrapper around cc_library.
    Adds options to the compilation command.
    '''
    default_copts = __flow_copts(name=name,
                                 sanitize=sanitize,
                                 debug_build=debug_build,
                                 asserts_as_exceptions=debug_build)

    default_linkopts = __flow_linkopts(name=name,
                                       sanitize=sanitize)

    native.cc_library(
        name=name,
        copts=copts + default_copts,
        linkopts=linkopts + default_linkopts,
        **kwargs
    )


def flow_cc_binary(name, copts=[], linkopts=[], sanitize=False, debug_build=False, **kwargs):
    '''
    A wrapper around cc_binary.
    Adds options to the compilation command.
    '''

    default_copts = __flow_copts(name=name,
                                 sanitize=sanitize,
                                 debug_build=debug_build,
                                 asserts_as_exceptions=debug_build)

    default_linkopts = __flow_linkopts(name=name,
                                       sanitize=sanitize)

    native.cc_binary(
        name=name,
        copts=copts + default_copts,
        linkopts=linkopts + default_linkopts,
        **kwargs
    )


def flow_cc_test(name, copts=[], linkopts=[], sanitize=False, debug_build=False, **kwargs):
    '''
    A wrapper around cc_test.
    Adds options to the compilation command.
    '''

    default_copts = __flow_copts(name=name,
                                 sanitize=sanitize,
                                 debug_build=debug_build,
                                 asserts_as_exceptions=True)

    default_linkopts = __flow_linkopts(name=name,
                                       sanitize=sanitize)
    native.cc_test(
        name=name,
        copts=copts + default_copts,
        linkopts=linkopts + default_linkopts,
        **kwargs
    )


def flow_cc_gtest(name, copts=[], linkopts=[], deps=[], sanitize=False, debug_build=False, **kwargs):
    '''
    A wrapper around cc_test for gtests
    Adds options to the compilation command.
    '''
    _GTEST_COPTS = [
        "-Iexternal/googletest/googletest/include",
        "-Iexternal/googletest/googlemock/include"
    ]


    _GTEST_DEPS = [
        "@googletest//:gtest",
        "@googletest//:gmock",
    ]

    flow_cc_test(name=name,
                 copts=_GTEST_COPTS + copts,
                 deps=_GTEST_DEPS + deps,
                 linkopts=linkopts,
                 sanitize=sanitize,
                 debug_build=debug_build,
                 **kwargs)
