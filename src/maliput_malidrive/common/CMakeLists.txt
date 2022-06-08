##############################################################################
# Library
##############################################################################

add_library(common
  common.cc
)
add_library(maliput_malidrive::common ALIAS common)
set_target_properties(common
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_common
)

target_include_directories(
  common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(common
  PUBLIC
    maliput::common
)

##############################################################################
# Installs
##############################################################################

install(TARGETS common
  EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
