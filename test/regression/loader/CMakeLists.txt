##############################################################################
# loader
##############################################################################

# Test binary name prefix, used by malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_LOADER_TEST_SOURCES
  loader_test.cc
)

maliput_malidrive_build_tests(${UNIT_LOADER_TEST_SOURCES})
