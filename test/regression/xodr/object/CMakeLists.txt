set(UNIT_TEST_XODR_OBJECT_SOURCES
  # TODO: object_test.cc
  repeat_test.cc
  outlines_test.cc
  skeleton_test.cc
  validity_test.cc
  material_test.cc
  parking_space_test.cc
  markings_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_XODR_OBJECT_SOURCES})
