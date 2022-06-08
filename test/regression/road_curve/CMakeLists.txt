##############################################################################
# Prep
##############################################################################

# Test binary name prefix, used by malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_TEST_ROAD_CURVE_SOURCES
  arc_ground_curve_test.cc
  cubic_polynomial_test.cc
  function_test.cc
  ground_curve_test.cc
  lane_offset_test.cc
  line_ground_curve_test.cc
  open_range_validator_test.cc
  piecewise_function_test.cc
  piecewise_ground_curve_test.cc
  road_curve_test.cc
  road_curve_offset_test.cc
  scaled_domain_function_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_ROAD_CURVE_SOURCES})
