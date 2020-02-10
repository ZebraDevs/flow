load("@flow//:flow_rules.bzl", "flow_cc_library")


flow_cc_library(
    name="flow",
    hdrs=glob([
        "include/**/*.h",
        "include/**/*.hpp",
    ]),
    include_prefix="flow",
    includes=["include"],
    strip_include_prefix="//flow/include",
    visibility=["//visibility:public"],
    deps=[],
)
