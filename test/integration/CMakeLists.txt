##############################################################################
# Prep
##############################################################################

# Test binary name prefix, used by malidrive_build_integration_based_tests
set(TEST_TYPE INTEGRATION)

##############################################################################
# Tests
##############################################################################

set(INTEGRATION_TEST_SOURCES
  extensive_queries_test.cc
)

# See #41 for more information about integration tests being disabled during tsan builds.
if (NOT THREAD_SANITIZER)
  malidrive_build_integration_based_tests(${INTEGRATION_TEST_SOURCES})
endif()
