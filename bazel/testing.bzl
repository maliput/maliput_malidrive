###############################################################################
# Loaders
###############################################################################

load("//:bazel/variables.bzl", "COPTS")

###############################################################################
# Testing Helpers
###############################################################################

def camel_to_snake(camel_case_name):
    res = [camel_case_name[0].lower()]
    for c in camel_case_name.elems()[1:]:
        if c in ('ABCDEFGHIJKLMNOPQRSTUVWXYZ'):
            res.append('_')
            res.append(c.lower())
        else:
            res.append(c)
    return ''.join(res)

# TODO(stonier) This is pretty suboptimal - creates a binary for each map.
#   The size = medium hammer is overkill for some tests too (bazel
#   test grumbles about this). If re-inventing, make sure to re-invent
#   the CMake test too.
#
def generate_integration_tests(sources, xodr_files):
    for xodr_file in xodr_files:
        camel_case_name = xodr_file[:-len(".xodr")]
        name = camel_to_snake(camel_case_name)
        native.cc_test(
            name="integration_test_" + name,
            srcs = sources,
            size = "large",
            timeout = "eternal",
            copts = COPTS,
            defines = [
                "XODR_FILE=\\\"resources/" + xodr_file + "\\\""
            ],
            data = ["//resources:all"],  # sledge hammer
            deps = [
                "//:maliput_malidrive",
                "//:utility",
                "@googletest//:gtest",
                "@maliput//:api",
                "@maliput//:common",
                "@maliput//:utility",
            ],
        )

# Not very bazel style (prefer individual test definitions)
def generate_unit_tests(sources):
    for source in sources:
        name = source[:-len(".cc")]
        native.cc_test(
            name = name,
            srcs = [source],
            size = "small",
            timeout = "moderate",
            copts = COPTS,
            linkopts = ["-lpthread"],
            defines = [
                "DEF_MALIDRIVE_RESOURCES=\\\"resources/\\\""
            ],
            data = ["//resources:all"],  # sledge hammer
            deps = [
                "//:maliput_malidrive",
                "//:utility",
                "@googletest//:gtest_main",
                "@maliput//:api",
                "@maliput//:base",
                "@maliput//:common",
                "@maliput//:geometry_base",
                "@maliput//:math",
                "@maliput//:plugin",
                "@maliput//:test_utilities",
                "//:private_test_headers",
            ],
        )
