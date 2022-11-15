load("@rules_foreign_cc//foreign_cc:defs.bzl", "configure_make")
load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

cc_binary(
    name = "main",
    srcs = ["main.cpp",],
    deps = [
        "//Dynamics:rigid_body",
        "//Dynamics:world",
        "//Dynamics:constraint",
        "//Dynamics:ball_joint",
        "//matplotlib:matplotlib",
        ],
)
