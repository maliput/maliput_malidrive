##############################################################################
# Prep
##############################################################################

add_subdirectory(base)
add_subdirectory(builder)
add_subdirectory(common)
add_subdirectory(loader)
add_subdirectory(plugin)
add_subdirectory(road_curve)
add_subdirectory(utility)
add_subdirectory(xodr)

##############################################################################
# Tests
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)


set(UNIT_TEST_SOURCES
  demo_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_SOURCES})
