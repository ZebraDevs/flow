load("@flow//:flow_rules.bzl", "flow_cc_gtest")


flow_cc_gtest(
    name="before",
    srcs=["before.cpp"],
    deps=["//flow:flow"],
    timeout="short",
)


flow_cc_gtest(
    name="closest_before",
    srcs=["closest_before.cpp"],
    deps=["//flow:flow"],
    timeout="short",
)

flow_cc_gtest(
    name="exact",
    srcs=["exact.cpp"],
    deps=["//flow:flow"],
    timeout="short",
)


flow_cc_gtest(
    name="latched",
    srcs=["latched.cpp"],
    deps=["//flow:flow"],
    timeout="short",
)


flow_cc_gtest(
    name="ranged",
    srcs=["ranged.cpp"],
    deps=["//flow:flow"],
    timeout="short",
)
