##############################################################################
# Prep
##############################################################################

# Test binary name prefix, used by malidrive_build_tests
set(TEST_TYPE UNIT_XODR)

##############################################################################
# Tests
##############################################################################

set(UNIT_TEST_XODR_SOURCES
  connection_test.cc
  db_manager_test.cc
  elevation_profile_test.cc
  geometry_test.cc
  header_test.cc
  junction_test.cc
  lane_link_test.cc
  lanes_test.cc
  lane_offset_test.cc
  lane_section_test.cc
  lane_test.cc
  lane_width_test.cc
  lateral_profile_test.cc
  parser_test.cc
  plan_view_test.cc
  reference_geometry_test.cc
  road_header_test.cc
  road_link_test.cc
  road_type_test.cc
  tools_test.cc
  unit_test.cc
  xodr_extract_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_XODR_SOURCES})
