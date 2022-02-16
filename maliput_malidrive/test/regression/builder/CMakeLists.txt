##############################################################################
# builder
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_BUILDER_TEST_SOURCES
  builder_tools_test.cc
  determine_tolerance_test.cc
  id_providers_test.cc
  phase_provider_builder_test.cc
  road_curve_factory_test.cc
  road_geometry_builder_test.cc
  road_network_builder_test.cc
  road_rulebook_builder_test.cc
  rule_registry_builder_test.cc
  rule_tools_test.cc
  road_geometry_configuration_test.cc
  road_network_configuration_test.cc
  simplify_geometries_test.cc
)

maliput_malidrive_build_tests(${UNIT_BUILDER_TEST_SOURCES})
