##############################################################################
# common
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_COMMON_TEST_SOURCES
  macros_test.cc
)

maliput_malidrive_build_tests(${UNIT_COMMON_TEST_SOURCES})
