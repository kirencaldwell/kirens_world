load("@rules_foreign_cc//foreign_cc:defs.bzl", "configure_make")
load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

cc_library(
    name = "utilities",
    srcs = ["utilities.cpp"],
    hdrs = ["utilities.h"],
    data = [],  # a cc_binary which we invoke at run time
    deps = [
    ],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "world",
    srcs = ["world.cpp"],
    hdrs = ["world.h"],
    data = [],  # a cc_binary which we invoke at run time
    deps = [
        ":rigid_body",
        ":constraint",
        ":utilities",
    ],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "ball_joint",
    srcs = ["ball_joint.cpp"],
    hdrs = ["ball_joint.h"],
    data = [],  # a cc_binary which we invoke at run time
    deps = [
        ":rigid_body",
        ":constraint",
        ":utilities",
    ],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "constraint",
    srcs = ["constraint.cpp"],
    hdrs = ["constraint.h"],
    data = [],  # a cc_binary which we invoke at run time
    deps = [
        ":rigid_body",
        ":utilities",
    ],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "rigid_body",
    srcs = ["rigid_body.cpp"],
    hdrs = ["rigid_body.h"],
    data = [],  # a cc_binary which we invoke at run time
    deps = [
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "test_constraint",
    size = "small",
    srcs = ["test_constraint.cpp",],
    deps = [
        "@com_google_googletest//:gtest_main",
        ":rigid_body",
        ":world",
        ":constraint",
        ":ball_joint",
        ],
)
cc_test(
    name = "test_world",
    size = "small",
    srcs = ["test_world.cpp",],
    deps = [
        "@com_google_googletest//:gtest_main",
        ":rigid_body",
        ":world",
        ":constraint",
        ":ball_joint",
        "//matplotlib:matplotlib",
        ],
)
cc_test(
    name = "test_rigid_body",
    size = "small",
    srcs = ["test_rigid_body.cpp",],
    deps = [
        "@com_google_googletest//:gtest_main",
        ":rigid_body",
        ],
)

