##############################################################################
# base
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_BASE_TEST_SOURCES
  lane_test.cc
  road_geometry_test.cc
  segment_test.cc
)

maliput_malidrive_build_tests(${UNIT_BASE_TEST_SOURCES})
