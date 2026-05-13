##############################################################################
# traffic_control_device
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_TEST_TRAFFIC_SIGNAL_SOURCES
  device_type_test.cc
  parser_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_TRAFFIC_SIGNAL_SOURCES})
