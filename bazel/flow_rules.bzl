def __flow_copts(name, debug_build=False):
    '''
    Common copts used in custom macros
    '''
    copts = ["-std=c++1y", "-Werror", "-Wall", "-Wextra", "-Wno-deprecated-declarations", "-Wno-unused-parameter", "-g"]

    if debug_build:
        # No optimizations
        copts += ["-O0", "-g3"]
    else:
        # Full optimizations; no debug asserts
        copts += ["-O3", "-DNDEBUG"]

    return copts


def __flow_linkopts(name):
    '''
    Common linkopts used in custom macros
    '''
    return []


def flow_cc_library(name, copts=[], linkopts=[], debug_build=False, **kwargs):
    '''
    A wrapper around cc_library.
    Adds options to the compilation command.
    '''
    default_copts = __flow_copts(name=name, debug_build=debug_build)

    default_linkopts = __flow_linkopts(name=name)

    native.cc_library(
        name=name,
        copts=copts + default_copts,
        linkopts=linkopts + default_linkopts,
        **kwargs
    )


def flow_cc_test(name, copts=[], linkopts=[], debug_build=True, **kwargs):
    '''
    A wrapper around cc_test.
    Adds options to the compilation command.
    '''

    default_copts = __flow_copts(name=name, debug_build=debug_build)

    default_linkopts = __flow_linkopts(name=name)
    native.cc_test(
        name=name,
        copts=copts + default_copts,
        linkopts=linkopts + default_linkopts,
        **kwargs
    )


def flow_cc_gtest(name, copts=[], linkopts=[], deps=[], debug_build=True, sanitize_build=True, **kwargs):
    '''
    A wrapper around cc_test for gtests
    Adds options to the compilation command.
    '''
    _GTEST_COPTS = [
        "-Iexternal/googletest/googletest/include",
    ]

    _GTEST_LINKOPTS = [
    ]

    _GTEST_DEPS = [
        "@googletest//:gtest",
    ]

    if sanitize_build:
        debug_build = True
        _GTEST_COPTS += [
            "-fsanitize=address",
            "-fsanitize-address-use-after-scope",
            "-DADDRESS_SANITIZER",
            "-fno-omit-frame-pointer",
        ]
        _GTEST_LINKOPTS += [
            "-fsanitize=address",
            "-static-libasan"
        ]

    flow_cc_test(name=name,
                 copts=_GTEST_COPTS + copts,
                 deps=_GTEST_DEPS + deps,
                 linkopts=_GTEST_LINKOPTS + linkopts,
                 debug_build=debug_build,
                 **kwargs)


def create_all_flow_cc_gtests(main, testcase_file_patterns, mode="optimized"):
    """
    Automatically creates gtests from a lists of files, located with glob
    """
    unit_test_files = native.glob(testcase_file_patterns)
    for file in unit_test_files:
        no_ext = file.split('.')[0]
        testcase = no_ext.split('/')[-1]
        flow_cc_gtest(
            name=testcase + "_" + mode,
            srcs=[file, main],
            deps=["//:flow"],
            debug_build=(mode in ("sanitized", "debug")),
            sanitize_build=(mode in ("sanitized",)),
            timeout="short",
        )
