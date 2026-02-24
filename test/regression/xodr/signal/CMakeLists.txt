set(UNIT_TEST_XODR_OBJECT_SOURCES
  board_test.cc
  controller_test.cc
  dependency_test.cc
  reference_test.cc
  sign_test.cc
  signal_reference_test.cc
  signal_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_XODR_OBJECT_SOURCES})
