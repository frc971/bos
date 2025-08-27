package(default_visibility = ["//visibility:public"])
load(":orin_toolchain_config.bzl", "orin_toolchain_config")

orin_toolchain_config(
    name = "orin_toolchain_config",
)

filegroup(
    name = "empty",
    srcs = [],
)

cc_toolchain(
  name = "orin_toolchain",
  all_files = ":empty",
  ar_files = ":empty",
  as_files = ":empty",
  compiler_files = ":empty",
  strip_files = ":empty",
  objcopy_files = ":empty",
  linker_files = ":empty",
  dwp_files = ":empty",
  toolchain_config = ":orin_toolchain_config"
)

toolchain(
  name = "cc_orin_toolchain",
  toolchain = ":orin_toolchain",
  toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
  exec_compatible_with = [
    "@platforms//cpu:arm64",
    "@platforms//os:macos",
  ],
  target_compatible_with = [
    "@platforms//cpu:arm64",
    "@platforms//os:macos",
  ],
)
