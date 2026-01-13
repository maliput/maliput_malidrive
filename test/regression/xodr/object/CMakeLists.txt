set(UNIT_TEST_XODR_OBJECT_SOURCES
  borders_test.cc
  bridge_test.cc
  markings_test.cc
  material_test.cc
  object_parser_test.cc
  object_reference_test.cc
  outlines_test.cc
  parking_space_test.cc
  repeat_test.cc
  skeleton_test.cc
  surface_test.cc
  tunnel_test.cc
  validity_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_XODR_OBJECT_SOURCES})
