load("@flow//:flow_rules.bzl", "flow_cc_library")


flow_cc_library(
    name="flow",
    hdrs=glob([
        "include/flow/**/*.h",
        "include/flow/**/*.hpp",
    ]),
    includes=["include"],
    strip_include_prefix="//include",
    visibility=["//visibility:public"],
    deps=[],
)
