##############################################################################
# plugin
##############################################################################

# Test binary name prefix, used by malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_PLUGIN_TEST_SOURCES
  road_network_plugin_test.cc
)

maliput_malidrive_plugin_tests(${UNIT_PLUGIN_TEST_SOURCES})
