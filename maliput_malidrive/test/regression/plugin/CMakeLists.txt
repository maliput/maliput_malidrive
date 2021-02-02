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

set (MALIPUT_MALIDRIVE_PLUGIN_LIBRARY_DIRS
  ${CMAKE_INSTALL_PREFIX}/../maliput_dragway/lib
  ${CMAKE_INSTALL_PREFIX}/../maliput_multilane/lib
)
# The following test uses the maliput::plugin::MaliputPluginManager entity to load all the availables plugins.
# Given that the LD_LIBRARY_PATH environment variable isn't correctly set up until setup.bash is sourced,
# and to avoid sourcing that file just to run one test we should append the paths.
# This allow to any other backend plugin to find its correspondant backend core library.
maliput_malidrive_build_tests(${UNIT_PLUGIN_TEST_SOURCES})
