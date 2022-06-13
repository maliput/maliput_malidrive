##############################################################################
# utility
##############################################################################

# Test binary name prefix, used by malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_UTILITY_TEST_SOURCES
  resources_test.cc
)

maliput_malidrive_utility_tests(${UNIT_UTILITY_TEST_SOURCES})
