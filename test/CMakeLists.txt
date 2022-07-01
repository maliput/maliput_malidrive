##############################################################################
# Tests
##############################################################################

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_gtest REQUIRED)

##############################################################################
# Macros
##############################################################################

macro(maliput_malidrive_plugin_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    set(RESOURCE_FILE_PREFIX ${PROJECT_SOURCE_DIR}/resources/)
    set(ROAD_NETWORK_PLUGIN ${CMAKE_INSTALL_PREFIX}/lib/maliput_plugins/)

    ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file} APPEND_LIBRARY_DIRS ${MALIPUT_MALIDRIVE_PLUGIN_LIBRARY_DIRS}
      TIMEOUT 240)

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(${BINARY_NAME}
        maliput::common
        maliput::plugin
        maliput_malidrive::base
        utility
    )

    # To avoid a false positive when running ubsan the symbols must be exported
    # See https://stackoverflow.com/questions/57361776/use-ubsan-with-dynamically-loaded-shared-libraries
    set_target_properties(${BINARY_NAME}
      PROPERTIES
        ENABLE_EXPORTS ON
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    target_compile_definitions(${BINARY_NAME}
      PRIVATE
        DEF_MALIDRIVE_RESOURCES="${RESOURCE_FILE_PREFIX}"
        DEF_ROAD_NETWORK_PLUGIN="${ROAD_NETWORK_PLUGIN}"
    )

  endforeach()
endmacro()

macro(maliput_malidrive_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    set(RESOURCE_FILE_PREFIX ${PROJECT_SOURCE_DIR}/resources/)

    ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file} TIMEOUT 240)

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(${BINARY_NAME}
        maliput::api
        maliput::base
        maliput::common
        maliput::geometry_base
        maliput::math
        maliput::test_utilities
        maliput_malidrive::base
        maliput_malidrive::builder
        maliput_malidrive::common
        maliput_malidrive::road_curve
        maliput_malidrive::test_utilities
        maliput_malidrive::xodr
        maliput_malidrive::xodr_extract
        pthread
        utility
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    target_compile_definitions(${BINARY_NAME}
      PRIVATE
        DEF_MALIDRIVE_RESOURCES="${RESOURCE_FILE_PREFIX}"
    )

  endforeach()
endmacro()

macro(maliput_malidrive_utility_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file} APPEND_LIBRARY_DIRS ${MALIPUT_MALIDRIVE_PLUGIN_LIBRARY_DIRS}
      TIMEOUT 240)

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(${BINARY_NAME}
        utility
    )

    # To avoid a false positive when running ubsan the symbols must be exported
    # See https://stackoverflow.com/questions/57361776/use-ubsan-with-dynamically-loaded-shared-libraries
    set_target_properties(${BINARY_NAME}
      PROPERTIES
        ENABLE_EXPORTS ON
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    target_compile_definitions(${BINARY_NAME}
      PRIVATE
        DEF_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    )

  endforeach()
endmacro()

# This macro generates a test target per each map on the list.
# Map file will be set using a CLI argument and it's the test responsibility
# to parse it.
macro(malidrive_build_integration_based_tests)
  set(XODR_FILE_PREFIX ${PROJECT_SOURCE_DIR}/resources/)
  set(XODR_FILE_PATHS
    ArcLane.xodr
    Highway.xodr
    LShapeRoad.xodr
    LShapeRoadVariableLanes.xodr
    LineMultipleSections.xodr
    LineVariableWidth.xodr
    LineVariableOffset.xodr
    ParkingGarageRamp.xodr
    RRFigure8.xodr
    RRLongRoad.xodr
    SingleLane.xodr
    SingleRoadComplexDescription.xodr
    SingleRoadComplexDescription2.xodr
    SingleRoadHighCoefficients.xodr
    SingleRoadNanValues.xodr
    SingleRoadNegativeWidth.xodr
    SingleRoadTinyGeometry.xodr
    SingleRoadTwoGeometries.xodr
    SShapeRoad.xodr
    SShapeSuperelevatedRoad.xodr
    StraightForward.xodr
    TShapeRoad.xodr
    Figure8.xodr
    Town01.xodr
    Town02.xodr
    Town03.xodr
    Town04.xodr
    Town05.xodr
    Town06.xodr
    Town07.xodr
  )

  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    # When integration tests are disabled, we just skip adding them.
    if(NOT MALIPUT_MALIDRIVE_INTEGRATION_TESTS)
      if(${TEST_TYPE} MATCHES "INTEGRATION")
        message(STATUS "Integration tests are disarmed, skipping: " ${GTEST_SOURCE_file})
        continue()
      endif()
    endif()

    foreach(XODR_MAP_file ${XODR_FILE_PATHS})
      string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
      set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME}_${XODR_MAP_file})

      ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file})

      target_include_directories(${BINARY_NAME}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/src
          ${PROJECT_SOURCE_DIR}/include
          ${PYTHON_INCLUDE_DIRS}
      )

      # Kind of an ugly catch-all bucket
      target_link_libraries(${BINARY_NAME}
          maliput::api
          maliput::common
          maliput::test_utilities
          maliput::utility
          maliput_malidrive::builder
          maliput_malidrive::loader
          maliput_malidrive::test_utilities
          utility
      )

      # Remove a warning in GTest.
      target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

      target_compile_definitions(${BINARY_NAME}
        PRIVATE
          XODR_FILE="${XODR_FILE_PREFIX}${XODR_MAP_file}"
      )

      # Extensive tests require more time when sanitizers are run due to extra
      # debug code introduced.
      set(TEST_TIMEOUT 300)
      if (SANITIZERS)
        set(TEST_TIMEOUT 340)
      endif()
      set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT ${TEST_TIMEOUT})
      unset(TEST_TIMEOUT)

    endforeach()
  endforeach()
endmacro()

##############################################################################
# Testing
##############################################################################

add_subdirectory(integration)
add_subdirectory(regression)
