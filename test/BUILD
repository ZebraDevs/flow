load("@flow//:bazel/flow_rules.bzl", "create_all_flow_cc_gtests")

create_all_flow_cc_gtests(
  main="gtest-main.cpp",
  testcase_file_patterns=["flow/*.cpp", "flow/driver/*.cpp", "flow/follower/*.cpp", "flow/utility/*.cpp"],
  mode="debug"
)

create_all_flow_cc_gtests(
  main="gtest-main.cpp",
  testcase_file_patterns=["flow/*.cpp", "flow/driver/*.cpp", "flow/follower/*.cpp", "flow/utility/*.cpp"],
  mode="sanitized"
)

create_all_flow_cc_gtests(
  main="gtest-main.cpp",
  testcase_file_patterns=["flow/*.cpp", "flow/driver/*.cpp", "flow/follower/*.cpp", "flow/utility/*.cpp"],
  mode="optimized"
)
